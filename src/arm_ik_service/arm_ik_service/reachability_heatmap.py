#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from sensor_msgs.msg import JointState
import numpy as np
import csv
import matplotlib.pyplot as plt
import seaborn as sns
import shutil

# base_link at world z=0.85. CT_EE_Z hover = 0.20 local = 1.05 world.
SWEEP_Z = 0.20

# Seed config matches coverage_planner's COUNTER_SEED — biases IK toward
# the wiping posture so boundary IKs converge instead of failing.
# Single seed (not multi) so the heatmap only marks cells the coverage
# planner can ALSO reach with smooth joint motion from this seed branch.
SEED_NAMES = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
SEED_POS   = [3.141592653589793, 0.5, -1.8, 0.0, -1.3, 0.0]


class ReachabilityHeatmap(Node):
    def __init__(self):
        super().__init__('reachability_heatmap')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /compute_ik...')
        self.get_logger().info('Reachability node ready.')

    def _seeded_state(self):
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = SEED_NAMES
        rs.joint_state.position = SEED_POS
        return rs

    def solve_ik(self, x, y, z, avoid_collisions=True):
        request = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'manipulator'
        ik_req.robot_state = self._seeded_state()
        ik_req.avoid_collisions = avoid_collisions
        ik_req.pose_stamped.header.frame_id = 'base_link'
        ik_req.pose_stamped.pose.position.x = x
        ik_req.pose_stamped.pose.position.y = y
        ik_req.pose_stamped.pose.position.z = z
        ik_req.pose_stamped.pose.orientation.x = 0.0
        ik_req.pose_stamped.pose.orientation.y = 1.0
        ik_req.pose_stamped.pose.orientation.z = 0.0
        ik_req.pose_stamped.pose.orientation.w = 0.0
        ik_req.timeout.sec = 1
        request.ik_request = ik_req
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return False
        return future.result().error_code.val == MoveItErrorCodes.SUCCESS

    def run_sweep(self):
        # 60×60cm counter patch on the LEFT side (y>0) where the side-mounted
        # mirror lives. Covers the full counter depth (x=0.30~0.90) and the
        # full +Y half-width (y=0~0.60) so the heatmap shows both M0609 reach
        # at the back corners AND the mirror keepout zone along y ≈ 0.58.
        x_start, x_end = 0.30, 0.90
        y_start, y_end = 0.00, 0.60
        resolution = 0.02

        xs = np.arange(x_start, x_end + resolution, resolution)
        ys = np.arange(y_start, y_end + resolution, resolution)
        total = len(xs) * len(ys)

        self.get_logger().info(f'Sweep 1/2: with collisions ({total} points)...')
        grid_collision = np.zeros((len(ys), len(xs)))
        count = 0
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                grid_collision[j, i] = 1.0 if self.solve_ik(x, y, SWEEP_Z, True) else 0.0
                count += 1
                if count % 100 == 0:
                    self.get_logger().info(f'  {count}/{total} ({100*count/total:.1f}%)')

        self.get_logger().info(f'Sweep 2/2: without collisions ({total} points)...')
        grid_kinematic = np.zeros((len(ys), len(xs)))
        count = 0
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                grid_kinematic[j, i] = 1.0 if self.solve_ik(x, y, SWEEP_Z, False) else 0.0
                count += 1
                if count % 100 == 0:
                    self.get_logger().info(f'  {count}/{total} ({100*count/total:.1f}%)')

        return xs, ys, grid_collision, grid_kinematic

    def save_csv(self, xs, ys, grid_collision, grid_kinematic,
                 path='/tmp/reachability.csv'):
        with open(path, 'w', newline='') as f:
            import csv as csv_mod
            writer = csv_mod.DictWriter(
                f, fieldnames=['x','y','z','reachable','obstacle_blocked']
            )
            writer.writeheader()
            for i, x in enumerate(xs):
                for j, y in enumerate(ys):
                    col = int(grid_collision[j, i])
                    kin = int(grid_kinematic[j, i])
                    # obstacle_blocked: kinematic reachable but collision blocked
                    obs = 1 if (col == 0 and kin == 1) else 0
                    writer.writerow({
                        'x': round(x, 3), 'y': round(y, 3),
                        'z': SWEEP_Z,
                        'reachable': col,
                        'obstacle_blocked': obs
                    })
        self.get_logger().info(f'CSV saved: {path}')

    def save_heatmap(self, xs, ys, grid_collision, grid_kinematic,
                     path='/tmp/reachability_heatmap.png'):
        # 3-color grid:
        # 1.0 = green  (reachable)
        # 0.5 = blue   (obstacle blocked: collision=False REACHABLE, collision=True BLOCKED)
        # 0.0 = red    (kinematic limit: both BLOCKED)
        display_grid = np.zeros((len(ys), len(xs)))
        obstacle_mask = np.zeros((len(ys), len(xs)), dtype=bool)

        for i in range(len(xs)):
            for j in range(len(ys)):
                col = grid_collision[j, i]
                kin = grid_kinematic[j, i]
                if col == 1.0:
                    display_grid[j, i] = 1.0   # green
                elif kin == 1.0:
                    display_grid[j, i] = 0.5   # obstacle blocked → blue
                    obstacle_mask[j, i] = True
                else:
                    display_grid[j, i] = 0.0   # red (kinematic limit)

        fig, ax = plt.subplots(figsize=(10, 8))
        sns.heatmap(
            display_grid,
            xticklabels=[f'{x:.2f}' for x in xs],
            yticklabels=[f'{y:.2f}' for y in ys],
            cmap='RdYlGn', vmin=0, vmax=1, ax=ax,
            cbar_kws={'label': 'Reachable (1) / Obstacle (0.5) / Kinematic Limit (0)'}
        )

        # Obstacle blocked → 파란색 overlay
        for i in range(len(xs)):
            for j in range(len(ys)):
                if obstacle_mask[j, i]:
                    ax.add_patch(plt.Rectangle(
                        (i, j), 1, 1,
                        fill=True, color='blue', alpha=0.8, zorder=10
                    ))

        for idx, label in enumerate(ax.get_xticklabels()):
            if idx % 5 != 0:
                label.set_visible(False)
        for idx, label in enumerate(ax.get_yticklabels()):
            if idx % 5 != 0:
                label.set_visible(False)

        ax.set_title(
            'M0609 Arm Reachability Heatmap\n'
            f'z={SWEEP_Z}m (base_link) = world z={SWEEP_Z+0.85:.2f}m\n'
            'Green=Reachable, Blue=Obstacle blocked, Red=Kinematic limit'
        )
        ax.set_xlabel('X (m, base_link)')
        ax.set_ylabel('Y (m, base_link)')
        plt.tight_layout()
        plt.savefig(path, dpi=150)
        plt.close()
        self.get_logger().info(f'Heatmap saved: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityHeatmap()

    xs, ys, grid_collision, grid_kinematic = node.run_sweep()

    reachable = int(grid_collision.sum())
    obstacle  = int(((grid_collision == 0) & (grid_kinematic == 1)).sum())
    kinematic = int(((grid_collision == 0) & (grid_kinematic == 0)).sum())
    total = len(xs) * len(ys)

    node.get_logger().info(
        f'Done. Reachable: {reachable}/{total} ({100*reachable/total:.1f}%) | '
        f'Obstacle blocked: {obstacle} | Kinematic limit: {kinematic}'
    )

    node.save_csv(xs, ys, grid_collision, grid_kinematic)
    node.save_heatmap(xs, ys, grid_collision, grid_kinematic)
    shutil.copy('/tmp/reachability_heatmap.png',
                '/home/kelly/arm_takehome_ws/deliverables/reachability_heatmap.png')
    shutil.copy('/tmp/reachability.csv',
                '/home/kelly/arm_takehome_ws/deliverables/reachability.csv')
    node.get_logger().info('Files copied to ~/arm_takehome_ws/deliverables/')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
