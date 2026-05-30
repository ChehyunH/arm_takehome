#!/usr/bin/env python3

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from stl import mesh as stl_mesh

# ── 전역 파라미터 (외부에서 참조 가능) ──
ROBOT_BASE_Z = 0.85    # world 기준 robot base 높이 (platform) — counter top과 같음, mirror reach 향상
CT_TOP       = 0.90    # world 기준 countertop 상단 높이
CT_DEPTH     = 0.60    # countertop 깊이 (x) — assignment spec: 120×60 cm
CT_WIDTH     = 1.20    # countertop 너비 (y) — assignment spec
CT_X_NEAR    = 0.25    # countertop near edge (robot 쪽, base_link 기준)
CT_X_FAR     = 0.85    # countertop far edge

class PlanningSceneNode(Node):
    def __init__(self):
        super().__init__('planning_scene_node')

        # world → base_link transform is published by M0609's robot_state_publisher
        # via the world_fixed joint in m0609.urdf.xacro (origin z=ROBOT_BASE_Z).
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /apply_planning_scene...')

        # Transient-local QoS so late-joining RViz subscribers still see the
        # platform marker (it's published once and never updated).
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # /visualization_marker_array is RViz's default MarkerArray topic
        # so an "Add -> MarkerArray" display picks it up with no config tweak.
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', latched)

        self.get_logger().info('Planning Scene Node ready.')

    def publish_visual_platform(self):
        """Publish a visual-only pedestal under the robot (not in collision
        scene; avoids intersecting link_1's collision mesh)."""
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'robot_platform'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = ROBOT_BASE_Z / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = ROBOT_BASE_Z
        m.color = ColorRGBA(r=0.30, g=0.30, b=0.32, a=1.0)
        m.frame_locked = True
        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().info('Published visual robot_platform marker.')

    def make_mesh(self, obj_id, stl_path, x, y, z, scale=1.0, yaw=0.0):
        raw = stl_mesh.Mesh.from_file(stl_path)
        if isinstance(scale, (list, tuple)):
            sx, sy, sz = float(scale[0]), float(scale[1]), float(scale[2])
        else:
            sx = sy = sz = float(scale)
        ros_mesh = Mesh()
        vertex_map = {}
        vertices = []
        for triangle in raw.vectors:
            indices = []
            for v in triangle:
                key = (round(float(v[0]), 8),
                       round(float(v[1]), 8),
                       round(float(v[2]), 8))
                if key not in vertex_map:
                    vertex_map[key] = len(vertices)
                    p = Point()
                    p.x, p.y, p.z = key[0]*sx, key[1]*sy, key[2]*sz
                    vertices.append(p)
                indices.append(vertex_map[key])
            mt = MeshTriangle()
            mt.vertex_indices = [indices[0], indices[1], indices[2]]
            ros_mesh.triangles.append(mt)
        ros_mesh.vertices = vertices
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = 'world'
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.z = math.sin(yaw / 2.0)
        obj.meshes = [ros_mesh]
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def remove_object(self, obj_id):
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = 'world'
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def make_box(self, obj_id, x, y, z, sx, sy, sz):
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = 'world'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def add_object(self, obj, color=None):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        if color is not None:
            oc = ObjectColor()
            oc.id = obj.id
            oc.color = ColorRGBA(r=float(color[0]), g=float(color[1]),
                                 b=float(color[2]), a=float(color[3]))
            scene.object_colors = [oc]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info(f'Added: {obj.id}')
        else:
            self.get_logger().error(f'FAILED: {obj.id}')

    def setup_scene(self):
        for old in ('countertop', 'countertop_near', 'countertop_far',
                    'countertop_left', 'countertop_right',
                    'cabinet', 'mirror', 'backsplash',
                    'sink', 'sink_bottom',
                    'sink_wall_near', 'sink_wall_far',
                    'sink_wall_left', 'sink_wall_right',
                    'faucet', 'object_0', 'faucet_neck', 'faucet_spout',
                    'robot_platform'):
            self.remove_object(old)

        CT_Z      = CT_TOP - 0.025
        CT_COLOR  = (0.93, 0.92, 0.88, 1.0)
        SINK_COLOR = (0.68, 0.68, 0.72, 1.0)

        # 싱크: 표준 컴팩트 키친 single bowl. 카운터 앞에서 10cm(한뼘),
        # 40cm 깊이 × 44cm 폭 × 20cm 깊이. 뒤 10cm 갭은 faucet mount용.
        SX0, SX1 = 0.35, 0.75
        SY0, SY1 = -0.22, 0.22
        S_CX = (SX0 + SX1) / 2
        S_CY = (SY0 + SY1) / 2
        S_W  = SX1 - SX0          # 40cm
        S_D  = SY1 - SY0          # 44cm
        SINK_DEPTH = 0.20          # 20cm 깊이

        # Robot mount pedestal: skipped as a collision object so it does not
        # intersect link_1's collision mesh (which extends below base_link).
        # In a real cell the robot is bolted to this pedestal; MoveIt should
        # not flag it as an obstacle.

        # ── 1. 카운터탑 (4조각 — 싱크 구멍 제외) ──
        # world x 기준: CT_X_NEAR=0.25 ~ CT_X_FAR=0.85 (base_link 기준)
        # world frame에서는 같은 값 (robot이 world origin에서 z만 올라감)
        self.add_object(self.make_box(
            'countertop_near',
            (CT_X_NEAR + SX0) / 2, 0.0, CT_Z,
            SX0 - CT_X_NEAR, CT_WIDTH, 0.05),
            color=CT_COLOR)
        self.add_object(self.make_box(
            'countertop_far',
            (SX1 + CT_X_FAR) / 2, 0.0, CT_Z,
            CT_X_FAR - SX1, CT_WIDTH, 0.05),
            color=CT_COLOR)
        self.add_object(self.make_box(
            'countertop_left',
            S_CX, (-CT_WIDTH/2 + SY0) / 2, CT_Z,
            S_W, SY0 + CT_WIDTH/2, 0.05),
            color=CT_COLOR)
        self.add_object(self.make_box(
            'countertop_right',
            S_CX, (SY1 + CT_WIDTH/2) / 2, CT_Z,
            S_W, CT_WIDTH/2 - SY1, 0.05),
            color=CT_COLOR)

        # ── 2. 싱크 basin ──
        wall_z = CT_TOP - SINK_DEPTH / 2
        T = 0.008
        self.add_object(self.make_box(
            'sink_bottom', S_CX, S_CY, CT_TOP - SINK_DEPTH,
            S_W, S_D, 0.006), color=SINK_COLOR)
        self.add_object(self.make_box(
            'sink_wall_near', SX0, S_CY, wall_z,
            T, S_D, SINK_DEPTH), color=SINK_COLOR)
        self.add_object(self.make_box(
            'sink_wall_far', SX1, S_CY, wall_z,
            T, S_D, SINK_DEPTH), color=SINK_COLOR)
        self.add_object(self.make_box(
            'sink_wall_left', S_CX, SY0, wall_z,
            S_W, T, SINK_DEPTH), color=SINK_COLOR)
        self.add_object(self.make_box(
            'sink_wall_right', S_CX, SY1, wall_z,
            S_W, T, SINK_DEPTH), color=SINK_COLOR)

        # ── 3. 하부 캐비닛 (바닥~countertop 아래) ──
        cab_h = CT_TOP - 0.05      # 0.85m
        self.add_object(self.make_box(
            'cabinet',
            (CT_X_NEAR + CT_X_FAR) / 2, 0.0, cab_h / 2,
            CT_DEPTH, CT_WIDTH, cab_h),
            color=(0.10, 0.14, 0.24, 1.0))

        # ── 4. 거울 — 측면(+Y) 벽에 mount (bathroom corner vanity 레이아웃)
        # 60cm wide (X) × 2cm depth (Y) × 90cm height (Z).
        # Mirror back face flush with counter side edge (y=+0.60), X aligned
        # with counter depth (x=0.25~0.85). Mirror bottom at counter top.
        self.add_object(self.make_box(
            'mirror',
            0.55, 0.59, CT_TOP + 0.45,
            0.60, 0.005, 0.90),
            color=(0.85, 0.95, 0.98, 0.85))

        # ── 5. 수전(faucet) STL mesh ──
        # Deck-mount: sink 뒤 rim에. 중심 x=0.73 (sink back 0.75에서 2cm 앞)
        self.add_object(self.make_mesh(
            'faucet',
            '/home/kelly/Downloads/object_0_mesh.stl',
            0.73, -0.05, CT_TOP + 0.08,
            scale=0.2,
            yaw=-math.pi / 2.0
        ), color=(0.80, 0.60, 0.15, 1.0))

        # Visual-only platform so the robot doesn't look like it floats.
        # Published as a Marker (not a CollisionObject) to avoid intersecting
        # link_1's collision mesh.
        self.publish_visual_platform()


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneNode()
    node.setup_scene()
    # Keep node alive briefly so the latched marker actually reaches RViz
    # before shutdown (transient_local needs the publisher to exist).
    import time
    time.sleep(2.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()