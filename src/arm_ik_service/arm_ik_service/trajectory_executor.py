#!/usr/bin/env python3
"""
Section 2 Trajectory Executor
Loads saved CSV trajectories and executes them on the Doosan M0609 arm via MoveIt.

Usage:
  ros2 run arm_ik_service trajectory_executor --ros-args \
    -p trajectory:=countertop_left
    # or: countertop_right, mirror_raster, mirror_spiral
"""

import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory


JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
]
N_JOINTS = len(JOINT_NAMES)

# Keys must match the filenames coverage_planner.save_csv() writes to /tmp/.
# The counter is planned as two halves (LEFT y>0, RIGHT y<0), so there is no
# single 'countertop' file — run the two halves separately.
TRAJECTORY_FILES = {
    'countertop_left':  '/tmp/countertop_left_trajectory.csv',
    'countertop_right': '/tmp/countertop_right_trajectory.csv',
    'mirror_raster':    '/tmp/mirror_raster_trajectory.csv',
    'mirror_spiral':    '/tmp/mirror_spiral_trajectory.csv',
}


def load_csv(path):
    """Load trajectory CSV and return list of (time_s, [joint_positions])."""
    points = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['time_s'])
            positions = [float(row[j]) for j in JOINT_NAMES if j in row]
            if len(positions) == 6:
                points.append((t, positions))
    return points


def build_joint_trajectory(points, scale=1.0):
    """Build JointTrajectory message with computed velocities."""
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES

    n = len(points)
    for i, (t, positions) in enumerate(points):
        pt = JointTrajectoryPoint()
        pt.positions = positions

        # Compute velocities via finite difference. Arrays MUST be the same
        # length as positions (6) — using 7 here caused an IndexError on the
        # first interior point and a velocities/positions length mismatch that
        # the controller rejected.
        if i == 0 or i == n-1:
            pt.velocities = [0.0] * N_JOINTS
        else:
            t_prev, p_prev = points[i-1]
            t_next, p_next = points[i+1]
            dt = (t_next - t_prev) * scale
            if dt > 1e-6:
                pt.velocities = [(p_next[j]-p_prev[j])/dt
                                 for j in range(N_JOINTS)]
            else:
                pt.velocities = [0.0] * N_JOINTS

        pt.accelerations = [0.0] * N_JOINTS

        scaled_t = t * scale
        pt.time_from_start = Duration(
            sec=int(scaled_t),
            nanosec=int((scaled_t - int(scaled_t)) * 1e9)
        )
        traj.points.append(pt)

    return traj


class TrajectoryExecutor(Node):

    def __init__(self):
        super().__init__('trajectory_executor')

        # Parameter: which trajectory to execute
        self.declare_parameter('trajectory', 'countertop_left')
        self.declare_parameter('time_scale', 1.0)   # slow down if > 1
        self.declare_parameter('use_moveit', True)   # True=MoveIt, False=controller

        traj_name  = self.get_parameter('trajectory').value
        time_scale = self.get_parameter('time_scale').value
        use_moveit = self.get_parameter('use_moveit').value

        self.get_logger().info(
            f'TrajectoryExecutor ready.\n'
            f'  trajectory : {traj_name}\n'
            f'  time_scale : {time_scale}\n'
            f'  use_moveit : {use_moveit}')

        # Load CSV
        if traj_name not in TRAJECTORY_FILES:
            self.get_logger().error(
                f'Unknown trajectory: {traj_name}. '
                f'Choose from: {list(TRAJECTORY_FILES.keys())}')
            return

        path = TRAJECTORY_FILES[traj_name]
        try:
            points = load_csv(path)
        except FileNotFoundError:
            self.get_logger().error(
                f'File not found: {path}\n'
                f'Run coverage_planner first to generate trajectory CSVs.')
            return

        self.get_logger().info(f'Loaded {len(points)} points from {path}')

        # Build trajectory
        joint_traj = build_joint_trajectory(points, scale=time_scale)

        if use_moveit:
            self._execute_moveit(joint_traj)
        else:
            self._execute_controller(joint_traj)

    def _execute_moveit(self, joint_traj):
        """Execute via MoveIt ExecuteTrajectory action."""
        client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for /execute_trajectory action...')
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/execute_trajectory not available!')
            self.get_logger().info('Try: use_moveit:=false for direct controller')
            return

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = joint_traj

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj

        self.get_logger().info(
            f'Sending trajectory to MoveIt '
            f'({len(joint_traj.points)} points, '
            f'{joint_traj.points[-1].time_from_start.sec}s)...')

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()

        # Wait for execution to complete
        total_time = (joint_traj.points[-1].time_from_start.sec +
                      joint_traj.points[-1].time_from_start.nanosec * 1e-9)
        self.get_logger().info(f'Expected duration: {total_time:.1f}s')

        rclpy.spin_until_future_complete(
            self, result_future,
            timeout_sec=total_time + 10.0)

        result = result_future.result()
        if result:
            err = result.result.error_code.val
            if err == 1:
                self.get_logger().info('Trajectory executed successfully!')
            else:
                self.get_logger().error(f'Execution failed: error_code={err}')
        else:
            self.get_logger().error('No result received!')

    def _execute_controller(self, joint_traj):
        """Execute directly via joint trajectory controller action."""
        client = ActionClient(
            self, FollowJointTrajectory,
            '/dsr_moveit_controller/follow_joint_trajectory')

        self.get_logger().info(
            'Waiting for /dsr_moveit_controller/follow_joint_trajectory...')
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Controller action not available!')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info(
            f'Sending to controller ({len(joint_traj.points)} points)...')

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        total_time = (joint_traj.points[-1].time_from_start.sec +
                      joint_traj.points[-1].time_from_start.nanosec * 1e-9)
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=total_time + 10.0)

        result = result_future.result()
        if result and result.result.error_code == 0:
            self.get_logger().info('Done!')
        else:
            self.get_logger().error(f'Failed: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    rclpy.shutdown()


if __name__ == '__main__':
    main()