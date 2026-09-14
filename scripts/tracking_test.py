#!/usr/bin/env python3
"""
End-effector tracking test for the sobits_teleop arm backends.

Works against BOTH the moveit_arm_controller plan-and-replace backend and
the MoveIt Servo backend — it only speaks the shared interface: *_target_link
TF + */moveit_track_enabled Bool.

Drives a pose path around each arm's start pose, on one or both arms at once
(--arms right,left ; non-primary arms are mirrored in y/yaw). Supports
sobit_home (dual arm) and sobit_light (single arm) via --robot, and multiple
path shapes via --path: the default Lissajous sweep, plus singularity-
crossing paths (roll_axis_cross, descend, yaw_sweep, radial). Logs per-arm
waypoint errors in SIM time (robust to any Gazebo real-time factor), counts
JointTrajectory messages published per arm (command throughput), and (when
available) MoveIt Servo status codes (singularity/collision/joint-bound
ticks) plus command/velocity based flip and drift metrics.

Requires: the robot sim (or real robot) up with move_group, and ONE tracking
backend running (moveit_arm_controller, or servo_arms.launch.py / servo.launch.py).
Run inside the robot's ROS environment (matching RMW + ROS_DOMAIN_ID):
  python3 tracking_test.py --robot sobit_home --arms right          # single arm
  python3 tracking_test.py --robot sobit_home --arms right,left     # both arms
  python3 tracking_test.py --robot sobit_light --path yaw_sweep     # sobit_light
Prints a JSON summary to stdout; per-waypoint CSVs to --out-prefix_<arm>.csv.
"""
import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory

try:
    from moveit_msgs.msg import ServoStatus
except ImportError:
    ServoStatus = None


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def q_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    )


def q_angle(a, b):
    d = abs(sum(x*y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def slerp(a, b, t):
    d = sum(x*y for x, y in zip(a, b))
    if d < 0:
        b = tuple(-x for x in b)
        d = -d
    if d > 0.9995:
        out = tuple(x + t*(y-x) for x, y in zip(a, b))
        n = math.sqrt(sum(x*x for x in out))
        return tuple(x/n for x in out)
    th = math.acos(min(1.0, d))
    s = math.sin(th)
    return tuple(math.sin((1-t)*th)/s*x + math.sin(t*th)/s*y for x, y in zip(a, b))


# ServoStatus.code -> name (mirrors moveit_msgs/msg/ServoStatus; falls back if unavailable).
SERVO_CODE_NAMES = {
    -1: 'INVALID', 0: 'NO_WARNING',
    1: 'DECELERATE_FOR_APPROACHING_SINGULARITY', 2: 'HALT_FOR_SINGULARITY',
    3: 'DECELERATE_FOR_LEAVING_SINGULARITY', 4: 'DECELERATE_FOR_COLLISION',
    5: 'HALT_FOR_COLLISION', 6: 'JOINT_BOUND',
}
HALT_FOR_SINGULARITY = 2
DECEL_APPROACH = 1
DECEL_LEAVE = 3
JOINT_BOUND = 6
DECEL_COLLISION = 4
HALT_COLLISION = 5

ROBOT_PRESETS = {
    'sobit_home': {
        'namespace': '/sobit_home',
        'default_arms': 'right,left',
        'arms': {
            'right': {'target_frame': 'right_target_link',
                      'ee_link': 'hand_right_end_effector_link',
                      'enable_topic': 'arm_right/moveit_track_enabled',
                      'traj_topic': 'arm_right_position_controller/joint_trajectory',
                      'status_topic': 'servo_arm_right/status',
                      'mirror': 1.0},
            'left':  {'target_frame': 'left_target_link',
                      'ee_link': 'hand_left_end_effector_link',
                      'enable_topic': 'arm_left/moveit_track_enabled',
                      'traj_topic': 'arm_left_position_controller/joint_trajectory',
                      'status_topic': 'servo_arm_left/status',
                      'mirror': -1.0},
        },
    },
    'sobit_light': {
        'namespace': '/sobit_light',
        'default_arms': 'arm',
        'arms': {
            'arm': {'target_frame': 'arm_target_link',
                    'ee_link': 'hand_end_effector_link',
                    'enable_topic': 'arm/moveit_track_enabled',
                    'traj_topic': 'arm_position_controller/joint_trajectory',
                    'status_topic': 'servo_arm/status',
                    'mirror': 1.0},
        },
    },
}


def parse_arm_spec(spec):
    parts = spec.split(':')
    if len(parts) not in (5, 6):
        raise ValueError(
            f'--arm-spec must be name:target_frame:ee_link:enable_topic:traj_topic[:mirror], got {spec!r}')
    name, target_frame, ee_link, enable_topic, traj_topic = parts[:5]
    mirror = float(parts[5]) if len(parts) == 6 else 1.0
    return name, {'target_frame': target_frame, 'ee_link': ee_link,
                  'enable_topic': enable_topic, 'traj_topic': traj_topic,
                  'status_topic': f'servo_{name}/status', 'mirror': mirror}


def lissajous_path(a, start, mirror, n):
    sx, sy, sz = start[:3]
    q0 = start[3:]
    lst = []
    for i in range(n):
        u = i / (n - 1)
        dx = a.amp_x * math.sin(2*math.pi*2*u)
        dy = mirror * a.amp_y * math.sin(2*math.pi*3*u + 1.0)
        dz = a.amp_z * math.sin(2*math.pi*1*u + 0.5)
        yaw = mirror * math.radians(a.yaw_deg) * math.sin(2*math.pi*1.5*u)
        pitch = math.radians(a.pitch_deg) * math.sin(2*math.pi*2.5*u + 0.7)
        q = q_mul(q0, q_from_euler(0.0, pitch, yaw))
        lst.append((sx+dx, sy+dy, sz+dz, *q))
    return lst


def _sweep(n, lo, hi):
    """Triangle sweep lo->hi->lo across n points."""
    half = (n - 1) / 2.0
    out = []
    for i in range(n):
        u = min(i, n - 1 - i) / half if half > 0 else 0.0
        out.append(lo + u * (hi - lo))
    return out


def named_path(path, o, mirror, n, line_from=None, line_to=None, hand='forward'):
    """Singularity-crossing paths built from base_frame->reference_frame origin o."""
    q_fwd = (0.0, 0.0, 0.0, 1.0)
    q_down = q_from_euler(0.0, math.pi/2, 0.0)
    lst = []
    if path == 'roll_axis_cross':
        for y in _sweep(n, -0.10, 0.10):
            lst.append((o.x+0.30, mirror*y, o.z, *q_fwd))
    elif path == 'descend':
        for z in _sweep(n, o.z, o.z-0.25):
            lst.append((o.x+0.20, 0.0, z, *q_down))
    elif path == 'yaw_sweep':
        for deg in _sweep(n, -30.0, 30.0):
            q = q_from_euler(0.0, 0.0, mirror*math.radians(deg))
            lst.append((o.x+0.25, 0.0, o.z-0.05, *q))
    elif path == 'radial':
        for r in _sweep(n, 0.28, 0.46):
            lst.append((o.x+r, 0.0, o.z-0.10, *q_fwd))
    elif path == 'line':
        # absolute base_frame endpoints (y mirrored per arm); --hand picks the orientation
        a = (line_from[0], mirror*line_from[1], line_from[2])
        b = (line_to[0], mirror*line_to[1], line_to[2])
        q = q_down if hand == 'down' else q_fwd
        for u in _sweep(n, 0.0, 1.0):
            lst.append(tuple(a[i] + u*(b[i]-a[i]) for i in range(3)) + q)
    else:
        raise ValueError(f'unknown path {path!r}')
    return lst


class DualTest(Node):
    def __init__(self, args):
        super().__init__('track_dual', namespace=args.namespace,
                         parameter_overrides=[rclpy.parameter.Parameter(
                             'use_sim_time', rclpy.Parameter.Type.BOOL, args.use_sim_time)])
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.arms = {}
        for name, spec in args.arm_cfgs.items():
            cfg = dict(spec)
            # transient_local + reliable: matches the servo_target_bridge subscriber
            # (compatible with the old controller's volatile subscriber too)
            enable_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            cfg['enable_pub'] = self.create_publisher(Bool, cfg['enable_topic'], enable_qos)
            cfg['target'] = None
            cfg['traj_count'] = 0
            cfg['joint_names'] = None
            cfg['prev_q'] = None
            cfg['max_cmd_dq'] = 0.0
            cfg['flip_count'] = 0
            cfg['q_first'] = None
            cfg['q_last'] = None
            cfg['max_joint_vel'] = 0.0
            cfg['prev_js'] = None
            cfg['status_hist'] = {}
            cfg['status_last'] = None
            cfg['status_prev'] = None
            cfg['halt_count'] = 0
            cfg['decel_ticks'] = 0
            cfg['joint_bound_ticks'] = 0
            cfg['collision_ticks'] = 0
            cfg['wp_halt'] = False
            cfg['wp_max_dq'] = 0.0

            def make_traj_cb(c):
                def cb(msg):
                    c['traj_count'] += 1
                    if not msg.points:
                        return
                    if c['joint_names'] is None:
                        c['joint_names'] = list(msg.joint_names)
                    q = list(msg.points[-1].positions)
                    if c['q_first'] is None:
                        c['q_first'] = q
                    c['q_last'] = q
                    if c['prev_q'] is not None and len(c['prev_q']) == len(q):
                        dqs = [abs(x-y) for x, y in zip(q, c['prev_q'])]
                        dq = max(dqs) if dqs else 0.0
                        c['max_cmd_dq'] = max(c['max_cmd_dq'], dq)
                        c['wp_max_dq'] = max(c['wp_max_dq'], dq)
                        if dq > 0.5:
                            c['flip_count'] += 1
                    c['prev_q'] = q
                return cb
            cfg['sub'] = self.create_subscription(
                JointTrajectory, cfg['traj_topic'], make_traj_cb(cfg), 10)

            def make_js_cb(c):
                def cb(msg):
                    if c['joint_names'] is None:
                        return
                    idxs = [msg.name.index(j) for j in c['joint_names'] if j in msg.name]
                    if not idxs:
                        return
                    if msg.velocity:
                        vmax = max(abs(msg.velocity[i]) for i in idxs)
                        c['max_joint_vel'] = max(c['max_joint_vel'], vmax)
                    elif c['prev_js'] is not None:
                        pt, pnames, ppos = c['prev_js']
                        dt = (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9) - pt
                        if dt > 0:
                            vmax = 0.0
                            for i in idxs:
                                nm = msg.name[i]
                                if nm in pnames:
                                    vmax = max(vmax, abs(msg.position[i]-ppos[pnames.index(nm)])/dt)
                            c['max_joint_vel'] = max(c['max_joint_vel'], vmax)
                    c['prev_js'] = (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                                     list(msg.name), list(msg.position))
                return cb
            cfg['js_sub'] = self.create_subscription(JointState, 'joint_states', make_js_cb(cfg), 10)

            if ServoStatus is not None:
                status_qos = QoSProfile(
                    depth=1, reliability=QoSReliabilityPolicy.RELIABLE)

                def make_status_cb(c):
                    def cb(msg):
                        c['status_prev'] = c['status_last']
                        c['status_last'] = msg.code
                        c['status_hist'][msg.code] = c['status_hist'].get(msg.code, 0) + 1
                        if msg.code == HALT_FOR_SINGULARITY and c['status_prev'] != HALT_FOR_SINGULARITY:
                            c['halt_count'] += 1
                            c['wp_halt'] = True
                        if msg.code in (DECEL_APPROACH, DECEL_LEAVE):
                            c['decel_ticks'] += 1
                        if msg.code == JOINT_BOUND:
                            c['joint_bound_ticks'] += 1
                        if msg.code in (DECEL_COLLISION, HALT_COLLISION):
                            c['collision_ticks'] += 1
                    return cb
                cfg['status_sub'] = self.create_subscription(
                    ServoStatus, cfg['status_topic'], make_status_cb(cfg), status_qos)

            self.arms[name] = cfg
        self.create_timer(0.01, self.broadcast)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def broadcast(self):
        for cfg in self.arms.values():
            if cfg['target'] is None:
                continue
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.args.base_frame
            t.child_frame_id = cfg['target_frame']
            x, y, z, qx, qy, qz, qw = cfg['target']
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(t)

    def ee_pose(self, cfg):
        tf = self.tf_buffer.lookup_transform(
            self.args.base_frame, cfg['ee_link'], rclpy.time.Time())
        tr = tf.transform
        return (tr.translation.x, tr.translation.y, tr.translation.z,
                tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)

    def lookup_origin(self, deadline):
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.base_frame, self.args.reference_frame, rclpy.time.Time())
                return tf.transform.translation
            except Exception:
                pass
        return None

    def errors(self, cfg):
        e = self.ee_pose(cfg)
        t = cfg['target']
        return math.dist(e[:3], t[:3]), q_angle(e[3:], t[3:])

    def spin_sim(self, dur, wall_cap=None):
        t0 = self.sim_now()
        wend = time.monotonic() + (wall_cap or dur / 0.02)
        while self.sim_now() - t0 < dur and time.monotonic() < wend:
            rclpy.spin_once(self, timeout_sec=0.005)

    def run(self):
        a = self.args
        # start poses
        deadline = time.monotonic() + 30
        starts = {}
        while time.monotonic() < deadline and len(starts) < len(self.arms):
            rclpy.spin_once(self, timeout_sec=0.05)
            for name, cfg in self.arms.items():
                if name in starts:
                    continue
                try:
                    starts[name] = self.ee_pose(cfg)
                except Exception:
                    pass
        if len(starts) < len(self.arms):
            print(json.dumps({'error': 'EE TF missing for some arm'}))
            return 1

        origin = None
        if a.path != 'lissajous':
            origin = self.lookup_origin(time.monotonic() + 30)
            if origin is None:
                print(json.dumps({'error': f'reference frame {a.reference_frame} TF missing'}))
                return 1

        # waypoints per arm (non-primary mirrored in y and yaw)
        N = a.n_points
        wps = {}
        for name, cfg in self.arms.items():
            m = cfg['mirror']
            if a.path == 'lissajous':
                wps[name] = lissajous_path(a, starts[name], m, N)
            else:
                wps[name] = named_path(a.path, origin, m, N, a.line_from, a.line_to, a.hand)
            cfg['target'] = starts[name]

        self.spin_sim(0.1, wall_cap=20)
        for _ in range(10):
            for cfg in self.arms.values():
                msg = Bool()
                msg.data = True
                cfg['enable_pub'].publish(msg)
            self.spin_sim(0.02, wall_cap=5)
        self.spin_sim(1.0, wall_cap=60)

        # Approach: glide from the latched pose to the first waypoint so the
        # sweep itself starts on target.
        if a.approach_sim > 0:
            t0 = self.sim_now()
            wend = time.monotonic() + a.seg_wall_cap * 4
            while time.monotonic() < wend:
                rclpy.spin_once(self, timeout_sec=0.005)
                f = (self.sim_now() - t0) / a.approach_sim
                if f >= 1.0:
                    break
                for name, cfg in self.arms.items():
                    p0, wp = starts[name], wps[name][0]
                    p = tuple(pv + f*(wv-pv) for pv, wv in zip(p0[:3], wp[:3]))
                    cfg['target'] = (*p, *slerp(p0[3:], wp[3:], f))
            for name, cfg in self.arms.items():
                cfg['target'] = wps[name][0]
            self.spin_sim(1.0, wall_cap=60)

        for cfg in self.arms.values():
            cfg['traj_count'] = 0
        per_wp = {name: [] for name in self.arms}
        sim0 = self.sim_now()

        prev = {name: self.arms[name]['target'] for name in self.arms}
        for idx in range(N):
            for cfg in self.arms.values():
                cfg['wp_halt'] = False
                cfg['wp_max_dq'] = 0.0
            seg_t0 = self.sim_now()
            wend = time.monotonic() + a.seg_wall_cap
            while time.monotonic() < wend:
                rclpy.spin_once(self, timeout_sec=0.005)
                f = (self.sim_now() - seg_t0) / a.hold_sim
                if f >= 1.0:
                    break
                for name, cfg in self.arms.items():
                    wp = wps[name][idx]
                    p = tuple(pv + f*(wv-pv) for pv, wv in zip(prev[name][:3], wp[:3]))
                    q = slerp(prev[name][3:], wp[3:], f)
                    cfg['target'] = (*p, *q)
            for name, cfg in self.arms.items():
                cfg['target'] = wps[name][idx]
                try:
                    pos, ang = self.errors(cfg)
                    per_wp[name].append(
                        (idx, self.sim_now()-sim0, pos, ang, cfg['status_last'],
                         cfg['wp_halt'], cfg['wp_max_dq']))
                except Exception:
                    pass
                prev[name] = wps[name][idx]

        track_dur = self.sim_now() - sim0
        # tail: let arms finish, record final error
        tail_t0 = self.sim_now()
        wend = time.monotonic() + a.seg_wall_cap * 4
        while self.sim_now() - tail_t0 < 1.5 and time.monotonic() < wend:
            rclpy.spin_once(self, timeout_sec=0.005)
        finals = {}
        for name, cfg in self.arms.items():
            try:
                pos, ang = self.errors(cfg)
                finals[name] = (pos, ang)
            except Exception:
                finals[name] = (None, None)

        for _ in range(5):
            for cfg in self.arms.values():
                msg = Bool()
                msg.data = False
                cfg['enable_pub'].publish(msg)
            self.spin_sim(0.02, wall_cap=5)

        out = {'arms': list(self.arms), 'n_waypoints': N,
               'track_sim_s': round(track_dur, 2), 'path': a.path, 'robot': a.robot}
        for name in self.arms:
            cfg = self.arms[name]
            pe = sorted(w[2]*100 for w in per_wp[name])
            ae = sorted(math.degrees(w[3]) for w in per_wp[name])
            n = len(pe)
            cnt = cfg['traj_count']
            roll_drift = 0.0
            if cfg['joint_names'] and cfg['q_first'] is not None and cfg['q_last'] is not None:
                for i, jn in enumerate(cfg['joint_names']):
                    if 'roll' in jn:
                        roll_drift = max(roll_drift, abs(cfg['q_last'][i] - cfg['q_first'][i]))
            status_hist = {str(k): v for k, v in cfg['status_hist'].items()}
            status_names = {str(k): SERVO_CODE_NAMES.get(k, 'UNKNOWN') for k in cfg['status_hist']}
            out[name] = {
                'pos_err_cm': {'mean': round(sum(pe)/n, 2), 'p50': round(pe[n//2], 2),
                               'p95': round(pe[int(n*.95)], 2), 'max': round(pe[-1], 2)},
                'ang_err_deg': {'mean': round(sum(ae)/n, 2), 'p50': round(ae[n//2], 2),
                                'p95': round(ae[int(n*.95)], 2), 'max': round(ae[-1], 2)},
                'final_pos_mm': (
                    round(finals[name][0]*1000, 1) if finals[name][0] is not None else None),
                'final_ang_deg': (
                    round(math.degrees(finals[name][1]), 2)
                    if finals[name][1] is not None else None),
                'traj_msgs': cnt,
                'traj_msgs_per_sim_s': round(cnt / track_dur, 1),
                'status_hist': status_hist,
                'status_names': status_names,
                'halt_count': cfg['halt_count'],
                'decel_ticks': cfg['decel_ticks'],
                'joint_bound_ticks': cfg['joint_bound_ticks'],
                'collision_ticks': cfg['collision_ticks'],
                'max_cmd_dq': round(cfg['max_cmd_dq'], 4),
                'flip_count': cfg['flip_count'],
                'roll_drift': round(roll_drift, 4),
                'max_joint_vel': round(cfg['max_joint_vel'], 4),
            }
            with open(a.out_prefix + f'_{name}.csv', 'w') as f:
                f.write('idx,sim_t,pos_err_m,ang_err_deg,status_code,halt,max_dq\n')
                for idx, t, p, ang, code, halt, dq in per_wp[name]:
                    f.write(f'{idx},{t:.3f},{p:.4f},{math.degrees(ang):.2f},'
                            f'{code if code is not None else ""},{int(halt)},{dq:.4f}\n')
        print(json.dumps(out))
        return 0


def build_arm_cfgs(args):
    preset = ROBOT_PRESETS[args.robot]
    cfgs = {name: dict(spec) for name, spec in preset['arms'].items()}
    for spec_str in args.arm_spec:
        name, spec = parse_arm_spec(spec_str)
        cfgs[name] = spec
    return cfgs


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', choices=['sobit_home', 'sobit_light'], default='sobit_home')
    p.add_argument('--arms', default=None)
    p.add_argument('--namespace', default=None)
    p.add_argument('--base-frame', default='base_footprint')
    p.add_argument('--use-sim-time', dest='use_sim_time', action='store_true', default=True)
    p.add_argument('--no-use-sim-time', dest='use_sim_time', action='store_false')
    p.add_argument('--arm-spec', action='append', default=[])
    p.add_argument('--path', choices=['lissajous', 'roll_axis_cross', 'descend',
                                       'yaw_sweep', 'radial', 'line'], default='lissajous')
    p.add_argument('--line-from', type=lambda v: tuple(map(float, v.split(','))), default=None,
                   help='x,y,z in base_frame for --path line')
    p.add_argument('--line-to', type=lambda v: tuple(map(float, v.split(','))), default=None)
    p.add_argument('--hand', choices=['forward', 'down'], default='forward',
                   help='fixed hand orientation for --path line')
    p.add_argument('--reference-frame', default='arm_shoulder_pitch_link')
    p.add_argument('--n-points', type=int, default=100)
    p.add_argument('--amp-x', type=float, default=0.06)
    p.add_argument('--amp-y', type=float, default=0.05)
    p.add_argument('--amp-z', type=float, default=0.07)
    p.add_argument('--yaw-deg', type=float, default=15.0)
    p.add_argument('--pitch-deg', type=float, default=12.0)
    p.add_argument('--hold-sim', type=float, default=0.2)
    p.add_argument('--approach-sim', type=float, default=0.0,
                   help='seconds to glide from the latched pose to waypoint 0 before sweeping')
    p.add_argument('--seg-wall-cap', type=float, default=30.0)
    p.add_argument('--out-prefix', default='/tmp/track_dual')
    args = p.parse_args()

    if args.namespace is None:
        args.namespace = ROBOT_PRESETS[args.robot]['namespace']
    if args.arms is None:
        args.arms = ROBOT_PRESETS[args.robot]['default_arms']
    all_cfgs = build_arm_cfgs(args)
    args.arm_cfgs = {name: all_cfgs[name] for name in args.arms.split(',')}

    rclpy.init()
    node = DualTest(args)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
