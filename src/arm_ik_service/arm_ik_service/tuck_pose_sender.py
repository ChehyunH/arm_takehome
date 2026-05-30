#!/usr/bin/env python3
"""
Send the M0609 to a 'tucked' home pose at startup so the arm sits cleanly
above the base instead of folding through the platform.

Runs once and exits. Intended to be launched on a TimerAction delay after
the controller spawner so the action server is live.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# Compact home facing the counter (world +X direction).
TUCK = [math.pi, -0.6, -1.4, 0.0, -1.1, 0.0]
DURATION_SEC = 3


def main(args=None):
    rclpy.init(args=args)
    node = Node('tuck_pose_sender')
    cli = ActionClient(
        node, FollowJointTrajectory,
        '/dsr_moveit_controller/follow_joint_trajectory')

    if not cli.wait_for_server(timeout_sec=30.0):
        node.get_logger().error(
            'FollowJointTrajectory action not available; skipping tuck.')
        rclpy.shutdown()
        return

    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = list(TUCK)
    pt.time_from_start = Duration(sec=DURATION_SEC)
    traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    f = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
    h = f.result()
    if not h or not h.accepted:
        node.get_logger().error('Tuck goal rejected.')
        rclpy.shutdown()
        return

    node.get_logger().info('Tuck goal accepted; waiting for completion...')
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(
        node, rf, timeout_sec=DURATION_SEC + 5.0)
    node.get_logger().info(f'Tuck complete: {TUCK}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()