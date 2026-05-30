#!/usr/bin/env python3
"""
Section 2: Surface Coverage Path Planning
- Sink-directed raster for countertop
- Top-down raster for mirror (faucet keepout, verified z range)
- Elliptical spiral for mirror (bonus)
- Continuous start state + IK seeding for smooth motion
"""

import math
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
import numpy as np

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest, MoveItErrorCodes
from sensor_msgs.msg import JointState


# ══════════════════════════════════════════════════════
#  Scene constants (WORLD frame)
# ══════════════════════════════════════════════════════
CT_TOP     = 0.90
PLATFORM_Z = 0.85

CT_X = (0.25, 0.85)            # 120cm wide × 60cm deep per assignment spec
CT_Y = (-0.60, 0.60)

SINK_X = (0.35, 0.75)
SINK_Y = (-0.22, 0.22)

FAUCET_X = (0.66, 0.80)
FAUCET_Y = (-0.07, 0.07)

MIR_Y_POS   = 0.59              # mirror box center y (back face flush with counter edge y=0.60)
MIR_HALF_DEPTH = 0.0025
MIR_FRONT_Y = MIR_Y_POS - MIR_HALF_DEPTH   # front face = 0.5875
MIR_CLEAR   = 0.12
MIR_Z_WORLD = (CT_TOP, CT_TOP + 0.90)       # mirror world z range 0.90~1.80
MIR_EE_Y    = 0.508             # flange 80mm from mirror front face

TOOL_W  = 0.10
TOOL_D  = 0.05
OVERLAP = 0.15
MARGIN  = 0.015

# Wiping surface positions (WORLD frame). Section 3 force controller engages
# contact by pushing further in surface-normal direction.
CT_EE_Z_WORLD = CT_TOP + 0.01    # counter wiping: 1cm above top → world z=0.91
MIR_Z_WIPE_WORLD = (CT_TOP, CT_TOP + 0.40)  # mirror sweep z range (world)

VEL_CT  = 0.20
VEL_MIR = 0.15
MAX_REACH = 0.88                # M0609 reach 0.855m with margin

# Mirror EE orientation — EE z-axis must point +Y (perpendicular to side
# mirror surface).  The free tool-roll DOF around that z-axis is captured
# by MIR_QUAT below.  The chosen value rolls the tool "right" axis to
# world +Z (brush up), which keeps the M0609 wrist in an IK branch where
# joint_4 changes <~0.8 rad across the rastered window (vs ~3 rad with
# the default tool-roll=0 which forced the wrist through a singularity).
# Quat = R_z(+pi/2) ∘ R_x(-pi/2), with axis (1,1,1)/√3 at 120°.
MIR_QUAT = (-0.5, -0.5, -0.5, 0.5)

# Mobile base position (world frame). Set from ROS param at runtime.
# Default = counter-wiping config. Mirror task sets base_y=0.30 for expanded reach.
BASE_X = 0.0
BASE_Y = 0.0
BASE_Z = 0.85

# Mirror raster ranges — expanded automatically when robot is shifted toward
# the side wall (BASE_Y >= 0.20). The reach filter prunes corners that still
# fall outside the M0609 sphere from the actual base position.
def get_mirror_ranges():
    """Return (x_range, z_range) rectangle that the planner rasters on
    the side mirror. With the C-shape arm configuration (elbow away from
    mirror), these ranges are deliberately conservative -- covering only
    the area the flange can comfortably reach without link collisions,
    rather than the full geometric reach sphere."""
    if BASE_Y >= 0.35:
        return (0.30, 0.80), (CT_TOP + 0.10, CT_TOP + 0.80)
    elif BASE_Y >= 0.20:
        return (0.30, 0.80), (CT_TOP + 0.10, CT_TOP + 0.80)
    return (0.35, 0.55), (CT_TOP, CT_TOP + 0.35)

def make_mirror_seed():
    """Pick the MIRROR_SEED based on robot base position.

    The seed's joint_1 is computed so the arm points at the mid-mirror
    waypoint from the current robot base — this keeps the IK in a single
    natural reach branch across the raster and avoids the elbow-flipping
    that happens when the seed is far from the target azimuth.

    At BASE_Y=0 (fixed base): we fall back to the empirically-tuned seed
    that gave the best Cartesian fraction on the side mirror.
    """
    if BASE_Y >= 0.20:
        # C-shape from RViz: -178°, 95°, -75°, 180°, 94°, 7°
        positions = [
            math.radians(-178),   # joint_1
            math.radians(95),     # joint_2
            math.radians(-75),    # joint_3
            math.radians(180),    # joint_4
            math.radians(94),     # joint_5
            math.radians(7),      # joint_6
        ]
    else:
        positions = [
            math.radians(-178), math.radians(95), math.radians(-75),
            math.radians(180), math.radians(94), math.radians(7),
        ]
    return {
        'names': ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
        'positions': positions,
    }
COUNTER_SEED = {
    'names': ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
    # Doosan M0609 seed for counter wiping (EE pointing down).
    'positions': [3.141592653589793, 0.5, -1.8, 0.0, -1.3, 0.0]
}

# LEFT counter (y > 0) is near the mirror — joint_1 is rotated toward the +Y
# side so the arm reaches FORWARD instead of sideways, keeping link_3 between
# the base and the target rather than swinging past the mirror.
COUNTER_LEFT_SEED = {
    'names': ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
    # joint_3=-1.0 (gentler bend) keeps link_3 between base and EE,
    # preventing it from swinging past the EE toward the mirror.
    'positions': [2.3, 0.3, -1.0, 0.0, -0.8, 0.0]
}


# ══════════════════════════════════════════════════════
#  Helper
# ══════════════════════════════════════════════════════
def pose_stamped(x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0,
                 frame='world'):
    """All waypoints are in WORLD frame so the same (x, y, z) values map to
    the correct EE pose regardless of where the robot base is mounted.
    MoveIt's TF tree (world → base_link via URDF world_fixed joint) handles
    the conversion for IK."""
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


def _reach_from_base(x, y, z):
    """Squared distance from current robot base position to a world point."""
    dx = x - BASE_X
    dy = y - BASE_Y
    dz = z - BASE_Z
    return dx*dx + dy*dy + dz*dz


def make_seed(d):
    rs = RobotState()
    rs.joint_state.name = d['names']
    rs.joint_state.position = d['positions']
    return rs


def _blocked(x, y, margin=MARGIN):
    in_sink = (SINK_X[0]-margin <= x <= SINK_X[1]+margin and
               SINK_Y[0]-margin <= y <= SINK_Y[1]+margin)
    in_fct  = (FAUCET_X[0]-margin <= x <= FAUCET_X[1]+margin and
               FAUCET_Y[0]-margin <= y <= FAUCET_Y[1]+margin)
    # Mirror at side wall (+Y) extends down to counter top. The arm's elbow
    # (link_3) swings 15-20 cm past the EE in +Y, so keep a generous buffer
    # (MIR_CLEAR) from the mirror front face — regardless of x — to prevent
    # link_3 from colliding with the mirror during execution.
    near_mir = (y >= MIR_FRONT_Y - MIR_CLEAR)
    # Reach from current robot base position (mobile base support).
    out_of_reach = _reach_from_base(x, y, CT_EE_Z_WORLD) > MAX_REACH*MAX_REACH
    return in_sink or in_fct or near_mir or out_of_reach


def _mir_blocked(y, margin=MARGIN):
    return FAUCET_Y[0]-margin <= y <= FAUCET_Y[1]+margin


def path_length(wps):
    total = 0.0
    for i in range(1, len(wps)):
        p, q = wps[i-1].pose.position, wps[i].pose.position
        total += math.sqrt((q.x-p.x)**2+(q.y-p.y)**2+(q.z-p.z)**2)
    return total


# ══════════════════════════════════════════════════════
#  Waypoint generators
# ══════════════════════════════════════════════════════
def countertop_raster_side(side):
    """Top-down raster for one side of the sink.

    Counter is split at y=0:
      side='left'  → y > 0
      side='right' → y < 0
    Snake direction: each row scans in Y (left-right wiping motion),
    advancing in X (depth) between rows. Top-down = outer Y edge → sink edge.
    Sink/faucet cells filtered by _blocked; rows just skip blocked waypoints.
    """
    step = TOOL_W * (1.0 - OVERLAP)
    if side == 'left':
        # y > 0: rows sweep from outer Y (+CT_Y) toward sink edge (y≈0)
        y_outer, y_inner = CT_Y[1]-MARGIN, MARGIN
    elif side == 'right':
        # y < 0: rows sweep from outer Y (-CT_Y) toward sink edge (y≈0)
        y_outer, y_inner = CT_Y[0]+MARGIN, -MARGIN
    else:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    # Outer loop: X (depth). Inner loop: Y (boustrophedon).
    xs = np.arange(CT_X[0]+MARGIN, CT_X[1]-MARGIN+1e-9, step)
    wps = []
    for i, x in enumerate(xs):
        if y_outer > y_inner:
            ys = np.arange(y_outer, y_inner-1e-9, -step)
        else:
            ys = np.arange(y_outer, y_inner+1e-9, step)
        if i % 2 == 1:
            ys = ys[::-1]
        wps.extend([pose_stamped(x, y, CT_EE_Z_WORLD)
                    for y in ys if not _blocked(x, y)])
    return wps


def countertop_raster_left_xsweep():
    """X-direction boustrophedon for the LEFT counter half (y > 0).

    Unlike the default Y-sweep used for RIGHT (which swings the arm in Y,
    causing link_3 to hit the mirror), this keeps Y FIXED within each stroke
    and sweeps in X (front-to-back).  The outer loop advances Y from the sink
    edge toward the mirror in small steps.  Because each stroke holds a
    constant Y, link_3 stays at a stable distance from the mirror — no
    sideways swing.  Advancing Y happens in controlled 8.5 cm steps, each of
    which can be validated for mirror clearance by _blocked().

    Sweep: starts near the sink (y ≈ 0) and advances toward the mirror
    (y → MIR_FRONT_Y - MIR_CLEAR).  This mirrors the RIGHT raster direction
    (outer-edge → sink) but flipped, giving the 'opposite direction' pattern.
    """
    step = TOOL_W * (1.0 - OVERLAP)
    # Outer loop: Y from past the sink edge toward mirror keepout
    y_start = max(MARGIN, SINK_Y[1] + MARGIN + 0.01)  # past sink (y≈0.245)
    ys = np.arange(y_start, MIR_FRONT_Y - MIR_CLEAR + 1e-9, step)
    # Squeeze one more row closer to the mirror if there's at least 4cm gap
    if len(ys) > 0:
        y_extra = ys[-1] + step * 0.55
        if y_extra < MIR_FRONT_Y - 0.03:
            ys = np.append(ys, y_extra)
    # Inner loop: X front-to-back boustrophedon
    wps = []
    for i, y in enumerate(ys):
        xs = np.arange(CT_X[0] + MARGIN, CT_X[1] - MARGIN + 1e-9, step)
        if i % 2 == 1:
            xs = xs[::-1]
        wps.extend([pose_stamped(x, y, CT_EE_Z_WORLD)
                    for x in xs if not _blocked(x, y)])
    return wps


def _mir_reachable(x, z):
    """True if a waypoint (x, MIR_EE_Y, z) world point is within MAX_REACH
    of the current robot base position."""
    return _reach_from_base(x, MIR_EE_Y, z) <= MAX_REACH*MAX_REACH


def mirror_raster():
    """Vertical-stroke raster on the side-wall mirror.

    X is the outer loop (column index), Z the inner boustrophedon
    (top↔bottom wipe). At a fixed X the wrist orientation barely
    changes along Z, so each vertical stroke stays inside one IK
    branch and the Cartesian planner can interpolate it without
    flipping. Waypoints are densified to ~3 cm in Z so per-row
    Cartesian fraction is high.

    We tested the textbook top-down pattern (Z outer, X inner =
    horizontal strokes) after tuning MIR_QUAT; even with the tuned
    wrist roll, horizontal strokes force joint_1 across the M0609
    branch boundary and the Cartesian planner drops to ~1 % fraction
    (7 pts). Vertical strokes keep joint_1 essentially constant
    inside a stroke (only Z varies), which is why we lead with this
    pattern despite it being less visually intuitive.

    EE quat = MIR_QUAT (tool z = +Y, tool "right" = world +Z).
    Reach filter drops corners outside the M0609 sphere."""
    sx = TOOL_W*(1.0-OVERLAP)
    sz_dense = 0.03  # densify Z so within-row Cartesian steps are small
    x_range, z_range = get_mirror_ranges()
    z0,z1 = z_range[0]+MARGIN, z_range[1]-MARGIN
    x0,x1 = x_range[0]+MARGIN, x_range[1]-MARGIN
    xs = np.arange(x0, x1+1e-9, sx)
    qx,qy,qz,qw = MIR_QUAT
    wps = []
    for i,x in enumerate(xs):
        zs = np.arange(z1, z0-1e-9, -sz_dense)
        if i%2==1: zs=zs[::-1]
        for z in zs:
            if not _mir_reachable(x, z):
                continue
            wps.append(pose_stamped(x, MIR_EE_Y, z, qx, qy, qz, qw))
    return wps


def mirror_spiral():
    """Inward elliptical spiral on the side-wall mirror. Centered in
    (X, Z) plane at MIR_EE_Y, spirals inward from outer edge. Unreachable
    corners are filtered."""
    qx,qy,qz,qw = MIR_QUAT
    x_range, z_range = get_mirror_ranges()
    z0,z1 = z_range[0]+MARGIN, z_range[1]-MARGIN
    x0r,x1r = x_range[0]+MARGIN, x_range[1]-MARGIN
    cx = (x0r+x1r)/2.0
    cz = (z0+z1)/2.0
    mrx = (x1r-x0r)/2.0
    mrz = (z1-z0)/2.0
    step = TOOL_W*(1.0-OVERLAP)
    n = max(3, int(max(mrx, mrz) / step) * 2)  # more turns = bigger spiral
    total = n*2.0*math.pi
    wps = []
    for theta in np.arange(0.0, total, 0.02):
        t = theta/total
        rx,rz = mrx*(1.0-t), mrz*(1.0-t)
        if rx<MARGIN or rz<MARGIN: break
        x = cx+rx*math.cos(theta)
        z = cz+rz*math.sin(theta)
        if not(x0r<=x<=x1r and z0<=z<=z1): continue
        if not _mir_reachable(x, z): continue
        wps.append(pose_stamped(x, MIR_EE_Y, z, qx, qy, qz, qw))
    return wps[::-1]   # inside-out: start from center, spiral outward


# ══════════════════════════════════════════════════════
#  Main node
# ══════════════════════════════════════════════════════
class CoveragePlanner(Node):

    def __init__(self):
        super().__init__('coverage_planner')

        # Read mobile-base position from ROS params and update module-level
        # globals so the waypoint generators and reach checks see them.
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_z', 0.85)
        # task: 'counter', 'mirror', or 'all' (default keeps legacy behavior)
        self.declare_parameter('task', 'all')
        global BASE_X, BASE_Y, BASE_Z
        BASE_X = float(self.get_parameter('base_x').value)
        BASE_Y = float(self.get_parameter('base_y').value)
        BASE_Z = float(self.get_parameter('base_z').value)
        self._task = str(self.get_parameter('task').value).lower()
        if self._task not in ('counter', 'mirror', 'all'):
            self.get_logger().warn(
                f"unknown task={self._task!r}, falling back to 'all'")
            self._task = 'all'
        self.get_logger().info(
            f'Robot base at world ({BASE_X:.2f}, {BASE_Y:.2f}, {BASE_Z:.2f}), '
            f'task={self._task}')

        self.cart_cli = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        while not self.cart_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /compute_cartesian_path...')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /compute_ik...')
        # transient_local QoS so late-joining RViz still sees the brush
        # strokes (markers are published once during planning then sit).
        latched = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/coverage_markers', latched)
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.get_logger().info('Coverage Planner ready.')

    def _js_cb(self, msg):
        self.current_joint_state = msg

    def get_ik_state(self, ps_msg, seed=None, avoid_collisions=True):
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = 'manipulator'
        ik.avoid_collisions = avoid_collisions
        ik.pose_stamped = ps_msg
        ik.timeout.sec = 2
        ik.robot_state = seed if seed else RobotState()
        if not seed:
            ik.robot_state.joint_state = JointState()
        req.ik_request = ik
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None or r.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        rs = RobotState()
        rs.joint_state = r.solution.joint_state
        return rs

    def get_ik_with_fallback(self, ps_msg, seeds):
        """Try IK with each seed in order; first success wins."""
        for s in seeds:
            rs = self.get_ik_state(ps_msg, s, avoid_collisions=True)
            if rs is not None:
                return rs
        # Last resort: try without collision checks
        for s in seeds:
            rs = self.get_ik_state(ps_msg, s, avoid_collisions=False)
            if rs is not None:
                self.get_logger().warn('IK only succeeded with collisions disabled')
                return rs
        return None

    def _plan_seg(self, wps, start, jt=2.0, avoid_collisions=True):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'  # waypoints are world-frame (mobile base support)
        req.group_name = 'manipulator'
        req.link_name = 'link_6'
        req.waypoints = [p.pose for p in wps]
        req.max_step = 0.005
        req.jump_threshold = jt
        req.avoid_collisions = avoid_collisions
        req.start_state = start
        fut = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if fut.result() is None: return None, 0.0
        return fut.result().solution, fut.result().fraction

    def compute_path(self, wps, seed=None, row_size=7, jt=2.0,
                     min_row_frac=0.5, gap_threshold=0.15,
                     joint_jump_tol=0.3, avoid_collisions=True):
        """
        Cartesian planning that *guarantees a single continuous trajectory*.

        The original implementation kept every row whose Cartesian fraction
        cleared `min_row_frac`, re-seeding `prev` from a fresh IK whenever a
        row in between failed. That re-seed could land in a different IK
        branch and produced joint discontinuities in the saved CSV. This
        version is honest about it: as soon as a row fails or the planner's
        first point would jump more than `joint_jump_tol` rad away from the
        previous saved point, the trajectory is *closed off* and returned.
        The CSV is therefore one smooth segment from start to first break —
        no stitching, no re-seeded branch flips, no fake continuity.
        """
        if not wps: return None, 0.0

        GAP_SQ = gap_threshold * gap_threshold
        segments = [[]]
        for w in wps:
            if segments[-1]:
                p = segments[-1][-1].pose.position
                q = w.pose.position
                d_sq = (q.x-p.x)**2 + (q.y-p.y)**2 + (q.z-p.z)**2
                if d_sq > GAP_SQ:
                    segments.append([])
            segments[-1].append(w)
        segments = [s for s in segments if s]

        all_pts, jnames = [], []
        planned_weighted = 0.0
        t_off = 0.0

        for s_idx, seg in enumerate(segments):
            prev = None
            rows = [seg[i:i+row_size] for i in range(0, len(seg), row_size)]
            for r_idx, row in enumerate(rows):
                if not row: continue
                if prev is not None:
                    start = prev
                else:
                    start = self.get_ik_with_fallback(
                        row[0], seeds=[seed, RobotState()])
                if start is None:
                    self.get_logger().warn(
                        f'Seg {s_idx} Row {r_idx}: IK failed, skip.')
                    continue
                traj, row_frac = self._plan_seg(row, start, jt,
                                                avoid_collisions)
                if traj is None or not traj.joint_trajectory.points \
                        or row_frac < min_row_frac:
                    self.get_logger().warn(
                        f'Seg {s_idx} Row {r_idx}: plan rejected '
                        f'(frac={row_frac:.2f}), skip.')
                    continue
                if not jnames: jnames = traj.joint_trajectory.joint_names
                pts = list(traj.joint_trajectory.points)
                # Joint-continuity check: drop the row if MoveIt picked a
                # different IK branch.  Use wrap-aware angular distance so
                # a ±π wrap (joint_1 going from +3.14 to -3.09) doesn't
                # look like a 6-rad jump — those are the same physical
                # configuration.  If only the ±π wrap differs we transparently
                # unwrap the new row's joint values to stay continuous in
                # the saved CSV.
                if all_pts:
                    last_q = list(all_pts[-1].positions)
                    first_q = list(pts[0].positions)
                    def _ad(a, b):
                        d = (b - a + math.pi) % (2*math.pi) - math.pi
                        return abs(d)
                    max_jump = max(_ad(a, b) for a, b in zip(first_q, last_q))
                    if max_jump > joint_jump_tol:
                        self.get_logger().warn(
                            f'Seg {s_idx} Row {r_idx}: IK branch flip '
                            f'({max_jump:.2f} rad), skip.')
                        continue
                    raw_max = max(abs(a-b) for a, b in zip(first_q, last_q))
                    if raw_max > math.pi:
                        offs = [0.0]*len(last_q)
                        for j, (a, b) in enumerate(zip(last_q, first_q)):
                            d = b - a
                            if d > math.pi:    offs[j] = -2*math.pi
                            elif d < -math.pi: offs[j] = 2*math.pi
                        for pt in pts:
                            pt.positions = tuple(p + offs[j]
                                                  for j, p in enumerate(pt.positions))
                for pt in pts:
                    t = (pt.time_from_start.sec
                         + pt.time_from_start.nanosec*1e-9 + t_off)
                    pt.time_from_start.sec = int(t)
                    pt.time_from_start.nanosec = int((t-int(t))*1e9)
                if pts:
                    last_t = (pts[-1].time_from_start.sec
                              + pts[-1].time_from_start.nanosec*1e-9)
                    t_off = last_t
                    lp = pts[-1]
                    prev = RobotState()
                    prev.joint_state.name = list(jnames)
                    prev.joint_state.position = list(lp.positions)
                all_pts.extend(pts)
                planned_weighted += len(row) * row_frac
            # Segment boundary: spatial gap before the next segment.
            # Pad time so the next segment's points don't share timestamps
            # with this segment's last point. The joint-jump filter above
            # keeps any branch flip out of the saved CSV — the time pad
            # itself is just a clock offset, not a synthesized motion.
            if s_idx + 1 < len(segments):
                t_off += 1.5

        if not all_pts:
            self.get_logger().error('No points planned!')
            return None, 0.0

        from moveit_msgs.msg import RobotTrajectory
        out = RobotTrajectory()
        out.joint_trajectory.joint_names = jnames or []
        out.joint_trajectory.points = all_pts
        frac = planned_weighted / len(wps)
        self.get_logger().info(
            f'  Segments: {len(segments)}, Fraction: {frac*100:.1f}% '
            f'({len(all_pts)} pts)')
        return out, frac

    @staticmethod
    def stamp(traj, vel):
        pts = traj.joint_trajectory.points
        if not pts: return traj
        t = 0.0
        pts[0].time_from_start.sec = 0
        pts[0].time_from_start.nanosec = 0
        for i in range(1, len(pts)):
            dq = np.array(pts[i].positions)-np.array(pts[i-1].positions)
            dt = max(float(np.linalg.norm(dq))*0.05, 0.02)
            t += dt
            pts[i].time_from_start.sec = int(t)
            pts[i].time_from_start.nanosec = int((t-int(t))*1e9)
        return traj

    def pub_markers(self, wps, ns, color, scale=0.008, surface_offset=(0,0,0)):
        """Publish thin LINE_STRIP connecting all waypoints."""
        ox, oy, oz = surface_offset
        def shifted(p):
            from geometry_msgs.msg import Point as _P
            q = _P(); q.x = p.x + ox; q.y = p.y + oy; q.z = p.z + oz
            return q
        ma = MarkerArray()
        ln = Marker()
        ln.header.frame_id = 'world'; ln.ns = ns; ln.id = 0
        ln.type = Marker.LINE_STRIP; ln.action = Marker.ADD
        ln.scale.x = scale
        ln.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.9)
        ln.pose.orientation.w = 1.0
        ln.points = [shifted(p.pose.position) for p in wps]
        ma.markers.append(ln)
        self.marker_pub.publish(ma)
        self.marker_pub.publish(ma)

    def report(self, wps, traj, area, name, vel=VEL_CT):
        plen = path_length(wps)
        step = TOOL_W*(1.0-OVERLAP)
        cov = min(100.0, len(wps)*step*step/area*100.0)
        exec_t = plen/vel
        self.get_logger().info(
            f'\n{"="*40}\n  {name}\n{"="*40}\n'
            f'  Waypoints : {len(wps)}\n'
            f'  Path      : {plen:.2f} m\n'
            f'  Coverage  : {cov:.1f} %\n'
            f'  Exec time : {exec_t:.1f} s (at {vel} m/s)\n'
            f'  Area      : {area:.4f} m2\n{"="*40}')
        return dict(name=name, waypoints=len(wps),
                    path_length_m=round(plen,3),
                    coverage_pct=round(cov,1),
                    exec_time_s=round(exec_t,1))

    @staticmethod
    def save_csv(traj, path='/tmp/trajectory.csv'):
        """Save trajectory to `path` (kept under /tmp/ so trajectory_executor
        can replay it) AND mirror to deliverables/ for the submission bundle.
        """
        import os as _os
        pts = traj.joint_trajectory.points
        js  = traj.joint_trajectory.joint_names
        deliv_path = _os.path.join(
            '/home/kelly/arm_takehome_ws/deliverables',
            _os.path.basename(path),
        )
        for out_path in (path, deliv_path):
            with open(out_path, 'w', newline='') as f:
                w = csv.writer(f); w.writerow(['time_s']+js)
                for pt in pts:
                    t = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9
                    w.writerow([round(t,4)]+[round(q,6) for q in pt.positions])
        print(f'Saved -> {path} ({len(pts)} pts) + deliverables/')

    def run(self):
        res = []
        cs = make_seed(COUNTER_SEED)
        cs_left = make_seed(COUNTER_LEFT_SEED)
        ms = make_seed(make_mirror_seed())

        # task='counter' → counter only (use at BASE_Y=0)
        # task='mirror'  → mirror only (use at BASE_Y=0.30)
        # task='all'     → both (default for legacy behavior)
        # When mobile base is enabled, picking the matching task at each
        # base position keeps IK targets in the reach-center → smooth joints.
        do_counter = self._task in ('counter', 'all')
        do_mirror  = self._task in ('mirror', 'all')
        self.get_logger().info(
            f'Task={self._task!r}: counter={do_counter}, mirror={do_mirror}')

        # Counter half-areas (each side = half of counter minus half-sink).
        ca_half = (CT_X[1]-CT_X[0]) * (CT_Y[1]-CT_Y[0]) / 2.0
        sa_half = (SINK_X[1]-SINK_X[0]) * (SINK_Y[1]-SINK_Y[0]) / 2.0
        ct_side_area = ca_half - sa_half

        # Marker surface projections (drops EE markers onto the contact face).
        ct_surf_offset  = (0.0, 0.0, -0.01)
        mir_surf_offset = (0.0, 0.0, 0.0)

        if do_counter:
            # A. Countertop LEFT (y > 0, top-down)
            self.get_logger().info('Countertop LEFT raster (y>0, Y-sweep)...')
            ct_l = countertop_raster_side('left')
            self.get_logger().info(f'  {len(ct_l)} waypoints')
            self.pub_markers(ct_l, 'countertop_left', (0.1, 0.9, 0.3),
                             surface_offset=ct_surf_offset)
            ct_l_t, _ = self.compute_path(ct_l, seed=cs_left, jt=3.0,
                                          min_row_frac=0.2,
                                          joint_jump_tol=1.0)
            if ct_l_t:
                ct_l_t = self.stamp(ct_l_t, VEL_CT)
                self.save_csv(ct_l_t, '/tmp/countertop_left_trajectory.csv')
            res.append(self.report(ct_l, ct_l_t, ct_side_area,
                                   'Countertop LEFT (top-down)', VEL_CT))

            # A2. Countertop RIGHT (y < 0, top-down)
            self.get_logger().info('Countertop RIGHT raster (y<0, top-down)...')
            ct_r = countertop_raster_side('right')
            self.get_logger().info(f'  {len(ct_r)} waypoints')
            self.pub_markers(ct_r, 'countertop_right', (0.1, 0.6, 0.9),
                             surface_offset=ct_surf_offset)
            ct_r_t, _ = self.compute_path(ct_r, seed=cs, jt=3.0,
                                          min_row_frac=0.2,
                                          joint_jump_tol=1.0)
            if ct_r_t:
                ct_r_t = self.stamp(ct_r_t, VEL_CT)
                self.save_csv(ct_r_t, '/tmp/countertop_right_trajectory.csv')
            res.append(self.report(ct_r, ct_r_t, ct_side_area,
                                   'Countertop RIGHT (top-down)', VEL_CT))

        if do_mirror:
            # B. Mirror spiral FIRST (bonus) — its end joint state seeds the
            #    raster so both strategies use the same arm configuration.
            self.get_logger().info('Mirror spiral...')
            sp = mirror_spiral()
            self.get_logger().info(f'  {len(sp)} waypoints')
            self.pub_markers(sp, 'mirror_spiral', (1.0, 0.4, 0.1),
                             surface_offset=mir_surf_offset)
            mir_x_range, mir_z_range = get_mirror_ranges()
            mirror_area = (mir_x_range[1] - mir_x_range[0]) * \
                          (mir_z_range[1] - mir_z_range[0])
            sp_t, _ = self.compute_path(sp, seed=ms, row_size=8, jt=2.0,
                                        min_row_frac=0.2)
            if sp_t:
                sp_t = self.stamp(sp_t, VEL_MIR)
                self.save_csv(sp_t, '/tmp/mirror_spiral_trajectory.csv')
            res.append(self.report(sp, sp_t, mirror_area,
                                   'Mirror Spiral Elliptical (bonus)', VEL_MIR))

            # Use spiral's end joint state as seed for raster (same config)
            ms_raster = ms
            if sp_t and sp_t.joint_trajectory.points:
                last_pt = sp_t.joint_trajectory.points[-1]
                ms_raster = RobotState()
                ms_raster.joint_state.name = list(
                    sp_t.joint_trajectory.joint_names)
                ms_raster.joint_state.position = list(last_pt.positions)

            # C. Mirror raster — seeded from spiral's configuration
            self.get_logger().info('Mirror raster...')
            mr = mirror_raster()
            self.get_logger().info(
                f'  {len(mr)} waypoints  x={mir_x_range} z={mir_z_range} '
                f'EE_y={MIR_EE_Y}  BASE_Y={BASE_Y}')
            self.pub_markers(mr, 'mirror_raster', (0.2, 0.5, 1.0),
                             surface_offset=mir_surf_offset)
            mr_t, _ = self.compute_path(mr, seed=ms_raster, row_size=8,
                                        jt=2.0, min_row_frac=0.2)
            if mr_t:
                mr_t = self.stamp(mr_t, VEL_MIR)
                self.save_csv(mr_t, '/tmp/mirror_raster_trajectory.csv')
            res.append(self.report(mr, mr_t, mirror_area,
                                   'Mirror Raster (Top-Down)', VEL_MIR))

        # Summary
        self.get_logger().info('\n'+'='*50+'\nSTRATEGY COMPARISON\n'+'='*50)
        for r in res:
            self.get_logger().info(
                f"  {r['name']:35s} | cov={r['coverage_pct']:5.1f}% | "
                f"len={r['path_length_m']:5.2f}m | t={r['exec_time_s']:5.1f}s")
        self.get_logger().info('='*50)
        return res


def main(args=None):
    rclpy.init(args=args)
    CoveragePlanner().run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()