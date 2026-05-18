#!/usr/bin/env python3
"""Activate the slam_toolbox lifecycle node via rclpy service calls.

Replaces the `ros2 lifecycle get/set --no-daemon` CLI approach that caused
DDS participant churn blocking slam_toolbox initialization on Pi Zero 2 W.
A persistent rclpy node waits for the slam_toolbox lifecycle services to
appear (up to --timeout seconds), then configure→activates the node.
In localization mode, optionally publishes an initial pose on /initialpose.

Exit codes: 0=success, 1=failure.
Writes status ("active" or "failed: <reason>") to --status-file if provided.
"""

import argparse
import math
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

try:
    from geometry_msgs.msg import PoseWithCovarianceStamped
    _HAS_GEOM = True
except ImportError:
    _HAS_GEOM = False

NODE_NAME = "/slam_toolbox"


def _write_status(path: str, msg: str) -> None:
    if not path:
        return
    try:
        with open(path, "w") as fh:
            fh.write(msg)
    except OSError as exc:
        print(f"[lifecycle_activate] WARNING: could not write status file: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status-file", default="",
                        help="File to write final status string into")
    parser.add_argument("--slam-mode", default="mapping",
                        help="Current SLAM mode (mapping or localization)")
    parser.add_argument("--seed-pose", nargs=3, type=float,
                        metavar=("X", "Y", "THETA"), default=None,
                        help="Initial pose to publish after activation (localization only)")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="Max seconds to wait for slam_toolbox services")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("rosie_lifecycle_activator")
    print(f"[lifecycle_activate] node started; waiting for {NODE_NAME} lifecycle services "
          f"(timeout={args.timeout:.0f}s, mode={args.slam_mode})")

    get_cli = node.create_client(GetState, f"{NODE_NAME}/get_state")
    chg_cli = node.create_client(ChangeState, f"{NODE_NAME}/change_state")

    # ── Wait for services ─────────────────────────────────────────────────────
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if get_cli.service_is_ready() and chg_cli.service_is_ready():
            break
        rclpy.spin_once(node, timeout_sec=2.0)
        elapsed = args.timeout - (deadline - time.monotonic())
        if int(elapsed) % 15 == 0:
            print(f"[lifecycle_activate]   still waiting... ({elapsed:.0f}s)")
    else:
        msg = f"failed: service timeout after {args.timeout:.0f}s"
        print(f"[lifecycle_activate] ERROR: {msg}")
        _write_status(args.status_file, msg)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ── Get current state ─────────────────────────────────────────────────────
    fut = get_cli.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if not fut.done() or fut.result() is None:
        msg = "failed: get_state timeout"
        print(f"[lifecycle_activate] ERROR: {msg}")
        _write_status(args.status_file, msg)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    state_id = fut.result().current_state.id
    print(f"[lifecycle_activate] current state id={state_id}")

    # ── Configure ─────────────────────────────────────────────────────────────
    if state_id == State.PRIMARY_STATE_UNCONFIGURED:
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        fut = chg_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        if not fut.done() or not fut.result().success:
            msg = "failed: configure transition"
            print(f"[lifecycle_activate] ERROR: {msg}")
            _write_status(args.status_file, msg)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        print("[lifecycle_activate] configured OK")
        state_id = State.PRIMARY_STATE_INACTIVE

    # ── Activate ──────────────────────────────────────────────────────────────
    if state_id == State.PRIMARY_STATE_INACTIVE:
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        fut = chg_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        if not fut.done() or not fut.result().success:
            msg = "failed: activate transition"
            print(f"[lifecycle_activate] ERROR: {msg}")
            _write_status(args.status_file, msg)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        print("[lifecycle_activate] activated OK")
    elif state_id == State.PRIMARY_STATE_ACTIVE:
        print("[lifecycle_activate] already active")
    else:
        print(f"[lifecycle_activate] WARNING: unexpected state_id={state_id}")

    # ── Seed initial pose (localization mode only) ────────────────────────────
    if (args.seed_pose is not None
            and args.slam_mode == "localization"
            and _HAS_GEOM):
        x, y, theta = args.seed_pose
        # slam_toolbox returns the activate service response before its internal
        # /initialpose subscriber is ready.  Wait for it to fully initialize
        # before publishing so the messages are not silently dropped.
        print("[lifecycle_activate] waiting 8 s for slam_toolbox to initialize "
              "before seeding initial pose…")
        deadline_settle = time.monotonic() + 8.0
        while time.monotonic() < deadline_settle:
            rclpy.spin_once(node, timeout_sec=0.5)
        print(f"[lifecycle_activate] seeding initial pose x={x} y={y} theta={theta}")
        pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        msg_pose = PoseWithCovarianceStamped()
        msg_pose.header.frame_id = "map"
        msg_pose.pose.pose.position.x = x
        msg_pose.pose.pose.position.y = y
        msg_pose.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg_pose.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg_pose.pose.covariance[0] = 0.25
        msg_pose.pose.covariance[7] = 0.25
        msg_pose.pose.covariance[35] = 0.068
        for i in range(15):
            pub.publish(msg_pose)
            rclpy.spin_once(node, timeout_sec=0.5)
            time.sleep(0.5)
            if i % 5 == 4:
                print(f"[lifecycle_activate] initial pose published {i + 1}/15")
        print("[lifecycle_activate] initial pose published OK")

    _write_status(args.status_file, "active")
    print("[lifecycle_activate] done")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
