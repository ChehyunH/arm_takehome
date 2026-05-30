#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from sensor_msgs.msg import JointState

class IKServiceNode(Node):
    def __init__(self):
        super().__init__('ik_service_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        self.get_logger().info('IK Service Node ready.')

    def solve_ik(self, x, y, z, avoid_collisions=True):
        request = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'manipulator'
        ik_req.robot_state = RobotState()
        ik_req.robot_state.joint_state = JointState()
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
            return None
        result = future.result()
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return list(result.solution.joint_state.position)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = IKServiceNode()

    # z별로 x range 테스트
    for z in [0.27, 0.30, 0.35, 0.40]:
        node.get_logger().info(f'--- z={z:.2f} (avoid_collisions=True) ---')
        for x in [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85]:
            result = node.solve_ik(x, 0.0, z, avoid_collisions=True)
            status = 'REACHABLE' if result else 'BLOCKED'
            node.get_logger().info(f'  x={x:.2f}: {status}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
