#!/usr/bin/env python3
"""
End-effector tracking test for the sobits_teleop arm backends.

Works against BOTH the moveit_arm_controller plan-and-replace backend and
the MoveIt Servo backend — it only speaks the shared interface: *_target_link
TF + */moveit_track_enabled Bool.

Drives a 100-waypoint pose path (3-axis Lissajous position + yaw/pitch
orientation sweep) around each arm's start pose, on one or both arms at once
(--arms right,left ; the left path is mirrored in y/yaw). Logs per-arm
waypoint errors in SIM time (robust to any Gazebo real-time factor) and
counts JointTrajectory messages published per arm (command throughput — the
direct indicator of cross-arm blocking).

Requires: the robot sim (or real robot) up with move_group, and ONE tracking
backend running (moveit_arm_controller, or servo_arms.launch.py).
Run inside the robot's ROS environment (matching RMW + ROS_DOMAIN_ID):
  python3 tracking_test.py --arms right              # single arm
  python3 tracking_test.py --arms right,left         # both arms
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
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory


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


ARMS = {
    'right': {'target_frame': 'right_target_link',
              'ee_link': 'hand_right_end_effector_link',
              'enable_topic': 'arm_right/moveit_track_enabled',
              'traj_topic': 'arm_right_position_controller/joint_trajectory',
              'mirror': 1.0},
    'left':  {'target_frame': 'left_target_link',
              'ee_link': 'hand_left_end_effector_link',
              'enable_topic': 'arm_left/moveit_track_enabled',
              'traj_topic': 'arm_left_position_controller/joint_trajectory',
              'mirror': -1.0},
}


class DualTest(Node):
    def __init__(self, args):
        super().__init__('track_dual', namespace='/sobit_home',
                         parameter_overrides=[rclpy.parameter.Parameter(
                             'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.arms = {}
        for name in args.arms.split(','):
            cfg = dict(ARMS[name])
            # transient_local + reliable: matches the servo_target_bridge subscriber
            # (compatible with the old controller's volatile subscriber too)
            enable_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            cfg['enable_pub'] = self.create_publisher(Bool, cfg['enable_topic'], enable_qos)
            cfg['target'] = None
            cfg['traj_count'] = 0

            def make_cb(c):
                def cb(_msg):
                    c['traj_count'] += 1
                return cb
            cfg['sub'] = self.create_subscription(
                JointTrajectory, cfg['traj_topic'], make_cb(cfg), 10)
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
            t.header.frame_id = 'base_footprint'
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
            'base_footprint', cfg['ee_link'], rclpy.time.Time())
        tr = tf.transform
        return (tr.translation.x, tr.translation.y, tr.translation.z,
                tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)

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

        # waypoints per arm (left mirrored in y and yaw)
        N = a.n_points
        wps = {}
        for name, cfg in self.arms.items():
            sx, sy, sz = starts[name][:3]
            q0 = starts[name][3:]
            m = cfg['mirror']
            lst = []
            for i in range(N):
                u = i / (N - 1)
                dx = a.amp_x * math.sin(2*math.pi*2*u)
                dy = m * a.amp_y * math.sin(2*math.pi*3*u + 1.0)
                dz = a.amp_z * math.sin(2*math.pi*1*u + 0.5)
                yaw = m * math.radians(a.yaw_deg) * math.sin(2*math.pi*1.5*u)
                pitch = math.radians(a.pitch_deg) * math.sin(2*math.pi*2.5*u + 0.7)
                q = q_mul(q0, q_from_euler(0.0, pitch, yaw))
                lst.append((sx+dx, sy+dy, sz+dz, *q))
            wps[name] = lst
            cfg['target'] = starts[name]

        self.spin_sim(0.1, wall_cap=20)
        for _ in range(10):
            for cfg in self.arms.values():
                msg = Bool()
                msg.data = True
                cfg['enable_pub'].publish(msg)
            self.spin_sim(0.02, wall_cap=5)
        self.spin_sim(1.0, wall_cap=60)

        for cfg in self.arms.values():
            cfg['traj_count'] = 0
        per_wp = {name: [] for name in self.arms}
        sim0 = self.sim_now()

        prev = {name: self.arms[name]['target'] for name in self.arms}
        for idx in range(N):
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
                    per_wp[name].append((idx, self.sim_now()-sim0, pos, ang))
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
               'track_sim_s': round(track_dur, 2)}
        for name in self.arms:
            pe = sorted(w[2]*100 for w in per_wp[name])
            ae = sorted(math.degrees(w[3]) for w in per_wp[name])
            n = len(pe)
            cnt = self.arms[name]['traj_count']
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
            }
            with open(a.out_prefix + f'_{name}.csv', 'w') as f:
                f.write('idx,sim_t,pos_err_m,ang_err_deg\n')
                for idx, t, p, ang in per_wp[name]:
                    f.write(f'{idx},{t:.3f},{p:.4f},{math.degrees(ang):.2f}\n')
        print(json.dumps(out))
        return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--arms', default='right,left')
    p.add_argument('--n-points', type=int, default=100)
    p.add_argument('--amp-x', type=float, default=0.06)
    p.add_argument('--amp-y', type=float, default=0.05)
    p.add_argument('--amp-z', type=float, default=0.07)
    p.add_argument('--yaw-deg', type=float, default=15.0)
    p.add_argument('--pitch-deg', type=float, default=12.0)
    p.add_argument('--hold-sim', type=float, default=0.2)
    p.add_argument('--seg-wall-cap', type=float, default=30.0)
    p.add_argument('--out-prefix', default='/tmp/track_dual')
    args = p.parse_args()

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
