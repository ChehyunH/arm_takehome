#!/usr/bin/env python3
"""
Section 3: Contact-Aware Wiping Controller

Simulated FT sensor + impedance-based force control.

State machine:
  FREE    -> |Fz| < F_CONTACT (2N): move at VEL_FREE
  CONTACT -> |Fz| >= F_CONTACT:     force control ON, maintain F_TARGET
  BACKOFF -> |Fz| > F_BACKOFF (15N): retract by BACKOFF_DIST

Force control (impedance):
  Countertop: adjust z (penetration depth)
  Mirror:     adjust x (distance from mirror)
  error   = F_target - |F_measured|
  offset -= Kp * error  (negative = move toward surface)

Surfaces:
  Countertop: F_target=10N +/-2N,   vel=0.15~0.25 m/s
  Mirror:     F_target= 6N +/-1.5N, vel=0.10~0.20 m/s

All coordinates in world frame.
"""

import math
import time
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, WrenchStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ══════════════════════════════════════════════════════
#  Scene constants (world frame)
# ══════════════════════════════════════════════════════
CT_TOP     = 0.90
PLATFORM_Z = 0.85

CT_X = (0.25, 0.85)
CT_Y = (-0.60, 0.60)

SINK_X = (0.35, 0.75)
SINK_Y = (-0.22, 0.22)

FAUCET_X = (0.70, 0.76)
FAUCET_Y = (-0.04, 0.04)

# Side-wall mirror (matches planning_scene + coverage_planner layout)
MIR_Y_POS   = 0.59               # mirror back face at counter side edge (y=0.60)
MIR_FRONT_Y = MIR_Y_POS - 0.0025  # mirror front face (5mm glass, y≈0.5875)
TOOL_LEN    = 0.10               # pad extends +EE_z direction (=+Y in world for mirror)

# Mobile base position (world frame). Default = counter-wiping config.
# Mirror task sets base_y=0.45 for expanded mirror coverage.
BASE_X = 0.0
BASE_Y = 0.0
BASE_Z = 0.85
MAX_REACH = 0.88


def _mirror_ranges():
    """Mirror X and Z sweep ranges — matches coverage_planner's
    get_mirror_ranges()."""
    if BASE_Y >= 0.35:
        return (0.30, 0.80), (CT_TOP + 0.10, CT_TOP + 0.80)
    if BASE_Y >= 0.20:
        return (0.30, 0.80), (CT_TOP + 0.10, CT_TOP + 0.80)
    return (0.35, 0.55), (CT_TOP, CT_TOP + 0.35)


# EE nominal positions (world frame)
CT_WIPE_Z  = CT_TOP - 0.012      # 12mm into surface → ~12N contact
# EE y so tool tip pushes ~8.5mm into mirror → ~6.8N at K_MIR=800 N/m
# tool tip y = EE_y + TOOL_LEN; contact starts at tool tip = MIR_FRONT_Y
MIR_EE_Y   = MIR_FRONT_Y - TOOL_LEN + 0.0085

# Tool
TOOL_W  = 0.10
TOOL_D  = 0.05
OVERLAP = 0.15
MARGIN  = 0.015

# ══════════════════════════════════════════════════════
#  Force control parameters
# ══════════════════════════════════════════════════════
F_CONTACT = 2.0
F_BACKOFF = 15.0

F_TARGET_CT  = 10.0
F_TARGET_MIR =  6.0
F_TOL_CT     =  2.0
F_TOL_MIR    =  1.5

VEL_FREE    = 0.25
VEL_CT_MIN  = 0.15
VEL_CT_MAX  = 0.25
VEL_MIR_MIN = 0.10
VEL_MIR_MAX = 0.20

KP_IMP = 0.00005   # m / (N * step)

BACKOFF_DIST = 0.02

SIM_HZ = 20.0
DT     = 1.0 / SIM_HZ


# ══════════════════════════════════════════════════════
#  Waypoint generators (world frame)
# ══════════════════════════════════════════════════════
def _ct_blocked(x, y):
    """Block sink interior, faucet, and waypoints outside the M0609 reach
    from the current robot base position."""
    in_s = (SINK_X[0]-MARGIN<=x<=SINK_X[1]+MARGIN and
            SINK_Y[0]-MARGIN<=y<=SINK_Y[1]+MARGIN)
    in_f = (FAUCET_X[0]-MARGIN<=x<=FAUCET_X[1]+MARGIN and
            FAUCET_Y[0]-MARGIN<=y<=FAUCET_Y[1]+MARGIN)
    # Mirror keepout — matches coverage_planner's MIR_CLEAR
    near_mir = (y >= MIR_FRONT_Y - 0.12)
    # Distance from mobile base to the wiping point.
    d_sq = (x-BASE_X)**2 + (y-BASE_Y)**2 + (CT_WIPE_Z-BASE_Z)**2
    return in_s or in_f or near_mir or d_sq > 0.82*0.82


def countertop_waypoints():
    """LEFT/RIGHT top-down raster (Y-snake) — matches Section 2's split.
    LEFT  (y>0): outer-loop X, inner-loop Y sweeping +CT_Y → 0.
    RIGHT (y<0): outer-loop X, inner-loop Y sweeping 0 → -CT_Y.
    """
    step = TOOL_W * (1.0 - OVERLAP)
    wz   = CT_WIPE_Z
    wps  = []
    xs = np.arange(CT_X[0]+MARGIN, CT_X[1]-MARGIN+1e-9, step)

    # LEFT half (y > 0)
    for i, x in enumerate(xs):
        ys = np.arange(CT_Y[1]-MARGIN, MARGIN-1e-9, -step)
        if i % 2 == 1: ys = ys[::-1]
        for y in ys:
            if not _ct_blocked(x, y):
                wps.append((float(x), float(y), wz))

    # RIGHT half (y < 0)
    for i, x in enumerate(xs):
        ys = np.arange(-MARGIN, CT_Y[0]+MARGIN-1e-9, -step)
        if i % 2 == 1: ys = ys[::-1]
        for y in ys:
            if not _ct_blocked(x, y):
                wps.append((float(x), float(y), wz))

    return wps


def mirror_waypoints():
    """Top-down raster on the side-wall mirror (XZ plane at fixed y).
    Mirror sweep extents depend on the mobile-base position via
    _mirror_ranges() — wider coverage when robot is shifted toward the
    side wall (BASE_Y >= 0.20)."""
    x_range, z_range = _mirror_ranges()
    step_x = TOOL_W*(1.0-OVERLAP)
    step_z = TOOL_D*(1.0-OVERLAP)
    z0,z1  = z_range[0]+MARGIN, z_range[1]-MARGIN
    zs     = np.arange(z1, z0-1e-9, -step_z)
    x0,x1  = x_range[0]+MARGIN, x_range[1]-MARGIN
    wps    = []
    for i,z in enumerate(zs):
        xs = np.arange(x0, x1+1e-9, step_x)
        if i%2==1: xs = xs[::-1]
        for x in xs:
            # Reach filter: drop waypoints out of M0609 sphere from base.
            d_sq = (x-BASE_X)**2 + (MIR_EE_Y-BASE_Y)**2 + (z-BASE_Z)**2
            if d_sq > MAX_REACH*MAX_REACH:
                continue
            wps.append((float(x), float(MIR_EE_Y), float(z)))
    return wps


# ══════════════════════════════════════════════════════
#  Simulated FT sensor
# ══════════════════════════════════════════════════════
class FTSensor:
    K_CT  = 1000.0
    K_MIR =  800.0
    NOISE =    0.20

    # Obstacles for BACKOFF demo
    OBSTACLES = [
        (0.45, 0.20, 0.04, 0.040),   # soap dispenser -> BACKOFF
        (0.35, -0.30, 0.03, 0.018),  # cup rim -> spike
    ]

    def __init__(self):
        self._rng = np.random.default_rng(42)

    def measure_ct(self, ex, ey, ez):
        bump = 0.0
        for cx,cy,r,h in self.OBSTACLES:
            d = math.sqrt((ex-cx)**2+(ey-cy)**2)
            if d < r:
                bump = max(bump, h*0.5*(1+math.cos(math.pi*d/r)))
        pen = max(0.0, (CT_TOP+bump) - ez)
        return float(-self.K_CT * pen + self._rng.normal(0, self.NOISE))

    def measure_mir(self, ex, ey, ez):
        # Tool tip y = ey + TOOL_LEN. Contact when tool tip exceeds mirror
        # front face (y=MIR_FRONT_Y); penetration grows with deeper EE.
        pen = max(0.0, (ey + TOOL_LEN) - MIR_FRONT_Y)
        return float(-self.K_MIR * pen + self._rng.normal(0, self.NOISE))


# ══════════════════════════════════════════════════════
#  State
# ══════════════════════════════════════════════════════
class State:
    FREE    = 'FREE'
    CONTACT = 'CONTACT'
    BACKOFF = 'BACKOFF'


# ══════════════════════════════════════════════════════
#  Controller
# ══════════════════════════════════════════════════════
class WipingController(Node):

    def __init__(self):
        super().__init__('wiping_controller')

        # Read mobile-base position + task from ROS params BEFORE generating
        # waypoints (mirror range depends on BASE_Y; task filters surfaces).
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_z', 0.85)
        self.declare_parameter('task', 'all')
        global BASE_X, BASE_Y, BASE_Z
        BASE_X = float(self.get_parameter('base_x').value)
        BASE_Y = float(self.get_parameter('base_y').value)
        BASE_Z = float(self.get_parameter('base_z').value)
        task = str(self.get_parameter('task').value).lower()
        if task not in ('counter', 'mirror', 'all'):
            self.get_logger().warn(
                f"unknown task={task!r}, falling back to 'all'")
            task = 'all'
        self._task = task

        self.ft_pub     = self.create_publisher(WrenchStamped, '/m0609/ft_sensor_sim', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/wiping_markers', 10)

        self.ft = FTSensor()

        # ── Connect to Section 2: use coverage_planner's waypoints ──
        import arm_ik_service.coverage_planner as cp
        cp.BASE_X, cp.BASE_Y, cp.BASE_Z = BASE_X, BASE_Y, BASE_Z

        def _ps_to_xyz(ps_list):
            return [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                    for p in ps_list]

        if task in ('counter', 'all'):
            ct_left  = _ps_to_xyz(cp.countertop_raster_side('left'))
            ct_right = _ps_to_xyz(cp.countertop_raster_side('right'))
            # Keep (x,y) pattern from coverage_planner but use force-tuned
            # CT_WIPE_Z so the FT simulator produces ~10N contact
            ct_all = ct_left + ct_right
            self.ct_wps = [(x, y, CT_WIPE_Z) for (x, y, _) in ct_all]
            self.get_logger().info(
                f'  Counter waypoints from coverage_planner: '
                f'LEFT {len(ct_left)} + RIGHT {len(ct_right)}, '
                f'z overridden to {CT_WIPE_Z:.3f}')
        else:
            self.ct_wps = []

        if task in ('mirror', 'all'):
            raw_mir = _ps_to_xyz(cp.mirror_spiral())
            # Keep (x,z) pattern from coverage_planner but use force-tuned
            # MIR_EE_Y so the FT simulator produces ~6N contact (not 15N)
            self.mir_wps = [(x, MIR_EE_Y, z) for (x, _, z) in raw_mir]
            self.get_logger().info(
                f'  Mirror waypoints from coverage_planner (spiral): '
                f'{len(self.mir_wps)}, y overridden to {MIR_EE_Y:.3f}')
        else:
            self.mir_wps = []
        self.all_wps = [('CT',  w) for w in self.ct_wps] + \
                       [('MIR', w) for w in self.mir_wps]
        if not self.all_wps:
            self.get_logger().error(
                f'No waypoints generated for task={task!r}.')
            raise SystemExit(1)

        self.wp_idx  = 0
        self.state   = State.FREE
        self.ee_pos  = list(self.all_wps[0][1])
        # Backoff target (3-tuple) — generic so it works for CT (+Z lift) and
        # for the side mirror (−Y retract).
        self.backoff_pos = (self.ee_pos[0], self.ee_pos[1],
                            self.ee_pos[2] + BACKOFF_DIST)

        # Separate impedance offsets per axis
        self._ct_z_off  = 0.0   # CT: z offset (push down on surface)
        self._mir_y_off = 0.0   # MIR: y offset (push +Y into side wall)

        self.log_t     = []
        self.log_fz    = []
        self.log_vel   = []
        self.log_state = []
        self.log_surf  = []

        mir_x_range, mir_z_range = _mirror_ranges()
        self.get_logger().info(
            f'WipingController ready.\n'
            f'  Robot base: ({BASE_X:.2f}, {BASE_Y:.2f}, {BASE_Z:.2f}) world\n'
            f'  CT:  {len(self.ct_wps)} wps  z={CT_WIPE_Z:.3f}m\n'
            f'  MIR: {len(self.mir_wps)} wps  y={MIR_EE_Y:.3f}m '
            f'x={mir_x_range} z={mir_z_range}')

    def _in_faucet(self, x, y, m=MARGIN*2):
        return (FAUCET_X[0]-m<=x<=FAUCET_X[1]+m and
                FAUCET_Y[0]-m<=y<=FAUCET_Y[1]+m)

    def _in_sink(self, x, y, m=MARGIN*2):
        return (SINK_X[0]-m<=x<=SINK_X[1]+m and
                SINK_Y[0]-m<=y<=SINK_Y[1]+m)

    def _handle_obstacle(self, surf, x, y):
        if surf=='CT' and self._in_faucet(x, y):
            skip = min(5, len(self.all_wps)-self.wp_idx-1)
            self.wp_idx += skip
            self.get_logger().warn(f'Faucet keepout -> skip {skip}')
            return True
        if surf=='CT' and self._in_sink(x, y):
            for s in range(1, min(20, len(self.all_wps)-self.wp_idx)):
                ns,(nx,ny,nz) = self.all_wps[self.wp_idx+s]
                if not self._in_sink(nx, ny):
                    self.wp_idx += s
                    self.get_logger().warn(f'Sink keepout -> skip {s}')
                    return True
            self.wp_idx = min(self.wp_idx+5, len(self.all_wps)-1)
            return True
        return False

    def step(self):
        if self.wp_idx >= len(self.all_wps):
            return False

        surf, target = self.all_wps[self.wp_idx]
        tx, ty, tz   = target
        ex, ey, ez   = self.ee_pos

        # Measure force
        if surf == 'CT':
            fz = self.ft.measure_ct(ex, ey, ez)
            f_target = F_TARGET_CT
            vel_min, vel_max = VEL_CT_MIN, VEL_CT_MAX
        else:
            fz = self.ft.measure_mir(ex, ey, ez)
            f_target = F_TARGET_MIR
            vel_min, vel_max = VEL_MIR_MIN, VEL_MIR_MAX

        abs_fz = abs(fz)

        # ── State machine ─────────────────────────────
        if self.state == State.BACKOFF:
            bx, by, bz = self.backoff_pos
            dx, dy, dz = bx - ex, by - ey, bz - ez
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            move = min(d, VEL_FREE * DT)
            if d > 1e-6:
                self.ee_pos[0] += dx / d * move
                self.ee_pos[1] += dy / d * move
                self.ee_pos[2] += dz / d * move
            vel = move / DT
            if d < 0.002:
                self.state = State.FREE
                self._ct_z_off  = 0.0
                self._mir_y_off = 0.0
                self.get_logger().warn('Backoff done -> FREE')

        elif abs_fz > F_BACKOFF:
            self.state = State.BACKOFF
            # Retract: CT lifts +Z, MIR pulls -Y (away from side wall).
            if surf == 'CT':
                self.backoff_pos = (ex, ey, ez + BACKOFF_DIST)
            else:
                self.backoff_pos = (ex, ey - BACKOFF_DIST, ez)
            self.get_logger().error(
                f'BACKOFF! {surf} |F|={abs_fz:.1f}N at '
                f'({ex:.2f},{ey:.2f},{ez:.3f})')
            vel = 0.0

        elif abs_fz >= F_CONTACT:
            self.state = State.CONTACT
            f_error = f_target - abs_fz

            if surf == 'CT':
                # Too low → push down (decrease z_off)
                # Too high → lift  up (increase z_off)
                self._ct_z_off -= KP_IMP * f_error
                self._ct_z_off  = float(np.clip(self._ct_z_off, -0.020, 0.020))
            else:
                # Too low → push toward mirror (+Y, increase y_off)
                # Too high → pull back (-Y, decrease y_off)
                self._mir_y_off += KP_IMP * f_error
                self._mir_y_off  = float(np.clip(self._mir_y_off, -0.020, 0.020))

            vel_ratio = 1.0 - abs(f_error) / (f_target + 1e-6)
            vel = float(np.clip(vel_ratio * vel_max, vel_min, vel_max))

        else:
            if self.state == State.CONTACT:
                self.get_logger().info('Contact lost -> FREE')
                self._ct_z_off  = 0.0
                self._mir_y_off = 0.0
            self.state = State.FREE
            vel = VEL_FREE

        # ── Move toward waypoint ──────────────────────
        if self.state != State.BACKOFF:
            nx_ = ex + (tx-ex)*0.1
            ny_ = ey + (ty-ey)*0.1
            if self._handle_obstacle(surf, nx_, ny_):
                vel = 0.0
            else:
                dist = math.sqrt((tx-ex)**2+(ty-ey)**2+(tz-ez)**2)
                if dist < 0.005:
                    self.wp_idx += 1
                    vel = 0.0
                else:
                    r = min(vel*DT/dist, 1.0)
                    if surf == 'CT':
                        self.ee_pos[0] += (tx-ex)*r
                        self.ee_pos[1] += (ty-ey)*r
                        self.ee_pos[2] += (tz + self._ct_z_off - ez)*r
                    else:  # MIR
                        self.ee_pos[0] += (tx-ex)*r
                        self.ee_pos[1] += (ty + self._mir_y_off - ey)*r
                        self.ee_pos[2] += (tz-ez)*r

        # Publish FT
        ws = WrenchStamped()
        ws.header.stamp = self.get_clock().now().to_msg()
        ws.header.frame_id = 'world'
        ws.wrench.force.z  = fz
        self.ft_pub.publish(ws)

        self.log_t.append(len(self.log_t)*DT)
        self.log_fz.append(fz)
        self.log_vel.append(vel)
        self.log_state.append(self.state)
        self.log_surf.append(surf)
        return True

    def publish_markers(self):
        ma = MarkerArray()

        # Planned path (orange) — snapped to surface
        ln = Marker()
        ln.header.frame_id='world'; ln.ns='plan'; ln.id=0
        ln.type=Marker.LINE_STRIP; ln.action=Marker.ADD
        ln.scale.x=0.004
        ln.color=ColorRGBA(r=1.0,g=0.6,b=0.0,a=0.6)
        for surf,w in self.all_wps:
            p=Point()
            if surf=='CT':
                p.x,p.y,p.z = w[0], w[1], CT_TOP+0.005
            else:
                # Mirror is on +Y side wall; snap markers to its front face
                p.x,p.y,p.z = w[0], MIR_FRONT_Y-0.005, w[2]
            ln.points.append(p)
        ma.markers.append(ln)

        # Completed path (blue)
        done = Marker()
        done.header.frame_id='world'; done.ns='done'; done.id=0
        done.type=Marker.LINE_STRIP; done.action=Marker.ADD
        done.scale.x=0.006
        done.color=ColorRGBA(r=0.2,g=0.5,b=1.0,a=0.9)
        for surf,w in self.all_wps[:self.wp_idx]:
            p=Point()
            if surf=='CT':
                p.x,p.y,p.z = w[0], w[1], CT_TOP+0.005
            else:
                # Mirror is on +Y side wall; snap markers to its front face
                p.x,p.y,p.z = w[0], MIR_FRONT_Y-0.005, w[2]
            done.points.append(p)
        if done.points:
            ma.markers.append(done)

        # EE sphere
        sp = Marker()
        sp.header.frame_id='world'; sp.ns='ee'; sp.id=0
        sp.type=Marker.SPHERE; sp.action=Marker.ADD
        sp.pose.position.x=self.ee_pos[0]
        sp.pose.position.y=self.ee_pos[1]
        sp.pose.position.z=self.ee_pos[2]
        sp.scale.x=sp.scale.y=sp.scale.z=0.030
        if self.state==State.FREE:
            sp.color=ColorRGBA(r=0.1,g=0.9,b=0.1,a=1.0)
        elif self.state==State.CONTACT:
            sp.color=ColorRGBA(r=1.0,g=0.8,b=0.0,a=1.0)
        else:
            sp.color=ColorRGBA(r=1.0,g=0.1,b=0.1,a=1.0)
        ma.markers.append(sp)
        self.marker_pub.publish(ma)

    def save_csv(self, path='/tmp/wiping_log.csv'):
        import os as _os
        deliv_path = _os.path.join(
            '/home/kelly/arm_takehome_ws/deliverables',
            _os.path.basename(path),
        )
        for out_path in (path, deliv_path):
            with open(out_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['time_s','fz_N','velocity_mps','state','surface'])
                for i in range(len(self.log_t)):
                    w.writerow([round(self.log_t[i],3),
                                round(self.log_fz[i],4),
                                round(self.log_vel[i],4),
                                self.log_state[i], self.log_surf[i]])
        self.get_logger().info(f'CSV -> {path} + deliverables/')

    def save_plots(self, out='/home/kelly/arm_takehome_ws/deliverables/wiping_plots.png'):
        if not HAS_MPL:
            return
        t   = np.array(self.log_t)
        fz  = np.abs(np.array(self.log_fz))
        v   = np.array(self.log_vel)
        su  = np.array(self.log_surf)
        sta = np.array(self.log_state)

        fig, axes = plt.subplots(2,1,figsize=(14,9),sharex=True)

        ax = axes[0]
        ct_m, mir_m = su=='CT', su=='MIR'
        ax.plot(t[ct_m],  fz[ct_m],  color='steelblue',   lw=1.2,
                label='|Fz| Countertop')
        ax.plot(t[mir_m], fz[mir_m], color='mediumpurple', lw=1.2,
                label='|Fx| Mirror')
        ax.axhline(F_CONTACT, color='orange', ls='--', lw=1.2,
                   label=f'Contact threshold ({F_CONTACT}N)')
        ax.axhline(F_BACKOFF, color='red',    ls='--', lw=1.2,
                   label=f'Backoff threshold ({F_BACKOFF}N)')
        ax.axhline(F_TARGET_CT,  color='steelblue',   ls=':', lw=1.0,
                   label=f'CT target ({F_TARGET_CT}N)')
        ax.axhline(F_TARGET_MIR, color='mediumpurple', ls=':', lw=1.0,
                   label=f'MIR target ({F_TARGET_MIR}N)')
        ax.fill_between(t, F_TARGET_CT-F_TOL_CT,   F_TARGET_CT+F_TOL_CT,
                        alpha=0.12, color='steelblue',
                        label=f'CT band (+-{F_TOL_CT}N)')
        ax.fill_between(t, F_TARGET_MIR-F_TOL_MIR, F_TARGET_MIR+F_TOL_MIR,
                        alpha=0.12, color='mediumpurple',
                        label=f'MIR band (+-{F_TOL_MIR}N)')
        ax.fill_between(t, 0, fz, where=fz>F_BACKOFF,
                        alpha=0.4, color='red', label='Backoff event')
        ax.fill_between(t, 0,
                        np.where((fz>F_CONTACT)&(fz<=F_BACKOFF), fz, 0),
                        where=(fz>F_CONTACT)&(fz<=F_BACKOFF),
                        alpha=0.15, color='orange', label='Contact region')
        ax.set_ylabel('Force (N)', fontsize=11)
        ax.set_title('Section 3: Contact-Aware Wiping Controller', fontsize=13)
        ax.legend(loc='upper right', fontsize=7, ncol=2)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)

        ax = axes[1]
        for s,c in [('FREE','green'),('CONTACT','orange'),('BACKOFF','red')]:
            mask = sta==s
            if mask.any():
                ax.fill_between(t, 0, v, where=mask,
                                alpha=0.3, color=c, label=s)
        ax.plot(t, v, color='black', lw=0.8, alpha=0.6, label='EE velocity')
        ax.axhline(VEL_CT_MAX,  color='steelblue',   ls='--', lw=1.0,
                   label=f'CT vel ({VEL_CT_MIN}~{VEL_CT_MAX}m/s)')
        ax.axhline(VEL_CT_MIN,  color='steelblue',   ls='--', lw=1.0)
        ax.axhline(VEL_MIR_MAX, color='mediumpurple', ls=':',  lw=1.0,
                   label=f'MIR vel ({VEL_MIR_MIN}~{VEL_MIR_MAX}m/s)')
        ax.axhline(VEL_MIR_MIN, color='mediumpurple', ls=':',  lw=1.0)
        ax.set_ylabel('EE Velocity (m/s)', fontsize=11)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.legend(loc='upper right', fontsize=7, ncol=2)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)

        plt.tight_layout()
        plt.savefig(out, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Plots -> {out}')

    def run(self):
        self.get_logger().info('Starting wiping simulation...')
        t0 = time.time()
        marker_t = 0.0

        # Stall guard: if wp_idx doesn't advance for STALL_MAX consecutive
        # ticks the obstacle handler is bouncing on a self-keepout (e.g.
        # an end-of-list waypoint that lives inside the sink rectangle).
        # We break out, save the partial trajectory and report cleanly.
        STALL_MAX = int(2.0 / DT)   # 2 seconds of no progress
        last_idx = -1
        stall = 0

        while rclpy.ok():
            if not self.step():
                break
            marker_t += DT
            if marker_t >= 0.25:
                self.publish_markers()
                marker_t = 0.0
            if self.wp_idx == last_idx:
                stall += 1
                if stall >= STALL_MAX:
                    self.get_logger().warn(
                        f'wp_idx={self.wp_idx} stalled '
                        f'{STALL_MAX*DT:.1f}s — ending sim.')
                    break
            else:
                stall = 0
                last_idx = self.wp_idx
            if self.wp_idx % 20 == 0 and self.log_fz:
                pct = 100.0*self.wp_idx/len(self.all_wps)
                self.get_logger().info(
                    f'  {pct:.0f}%  state={self.state}'
                    f'  Fz={self.log_fz[-1]:.1f}N'
                    f'  v={self.log_vel[-1]:.3f}m/s'
                    f'  surf={self.log_surf[-1]}')
            time.sleep(DT)

        elapsed = time.time()-t0
        n = max(len(self.log_state), 1)
        ct_c = sum(1 for s in self.log_state if s==State.CONTACT)
        bo   = sum(1 for s in self.log_state if s==State.BACKOFF)

        ct_fz  = [abs(f) for f,s,su in
                  zip(self.log_fz,self.log_state,self.log_surf)
                  if s==State.CONTACT and su=='CT']
        mir_fz = [abs(f) for f,s,su in
                  zip(self.log_fz,self.log_state,self.log_surf)
                  if s==State.CONTACT and su=='MIR']

        self.get_logger().info(
            f'\n{"="*50}\n  WIPING COMPLETE\n{"="*50}\n'
            f'  Elapsed        : {elapsed:.1f}s\n'
            f'  WPs done       : {self.wp_idx}/{len(self.all_wps)}\n'
            f'  Contact frac   : {100*ct_c/n:.1f}%\n'
            f'  Backoff steps  : {bo}\n'
            f'  CT  avg |Fz|   : {np.mean(ct_fz) if ct_fz else 0:.2f}N'
            f'  (target {F_TARGET_CT}+/-{F_TOL_CT}N)\n'
            f'  MIR avg |Fz|   : {np.mean(mir_fz) if mir_fz else 0:.2f}N'
            f'  (target {F_TARGET_MIR}+/-{F_TOL_MIR}N)\n'
            f'  Max |Fz|       : {max(abs(f) for f in self.log_fz):.2f}N\n'
            f'{"="*50}')

        suffix = self._task  # 'counter', 'mirror', or 'all'
        self.save_csv(f'/tmp/wiping_log_{suffix}.csv')
        self.save_plots(f'/home/kelly/arm_takehome_ws/deliverables/wiping_plots_{suffix}.png')


def main(args=None):
    rclpy.init(args=args)
    WipingController().run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
