# -*- coding: utf-8 -*-
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 moieboy9999 - Micro Dog
"""
Derive params/robot_params.yaml from the raw Autodesk Inventor dumps in ../params/*.json.

Everything in the CAD dumps is expressed in Inventor's database units
(cm, kg, kg*cm^2) in the *assembly* coordinate frame of 전체.iam:

    CAD frame :  +X = forward,  +Y = up,     +Z = robot's right
    URDF frame:  +X = forward,  +Y = left,   +Z = up      (ROS convention)

so the fixed remap between them is

    p_urdf = M @ p_cad ,   M = [[1, 0, 0],
                                [0, 0,-1],
                                [0, 1, 0]]

Joint zero (q = 0) is the CAD assembly pose, i.e. every Dynamixel sitting at
its centre position (2048 ticks / 180 deg).  In that pose the thigh points
straight backwards and the shank points straight down.

Run:  python derive_params.py
"""
import io
import json
import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAD = os.path.join(ROOT, 'params')

# CAD -> URDF axis remap
M = np.array([[1.0, 0.0, 0.0],
              [0.0, 0.0, -1.0],
              [0.0, 1.0, 0.0]])

CM = 0.01          # cm -> m
INERTIA = 1e-4     # kg*cm^2 -> kg*m^2


# --------------------------------------------------------------------------
# raw CAD data
# --------------------------------------------------------------------------
def load():
    tree = json.load(io.open(os.path.join(CAD, 'tree.json'), encoding='utf-8-sig'))
    mp = json.load(io.open(os.path.join(CAD, 'massprops.json'), encoding='utf-8-sig'))
    subs = json.load(io.open(os.path.join(CAD, 'thigh_subs.json'), encoding='utf-8-sig'))['subs']
    by_parent = {}
    for s in subs:
        by_parent.setdefault(s['parent'], []).append(s)
    return {r['path']: r for r in tree}, {r['name']: r for r in mp}, by_parent


def occ_pose(row):
    t = row['t']
    R = np.array([[t[0], t[1], t[2]],
                  [t[4], t[5], t[6]],
                  [t[8], t[9], t[10]]])
    p = np.array([t[3], t[7], t[11]])
    return R, p


def occ_bbox(row):
    lo = np.array([float(x) for x in row['rb_min'].split(',')])
    hi = np.array([float(x) for x in row['rb_max'].split(',')])
    return lo, hi


def tensor(I6):
    """Inventor XYZMomentsOfInertia -> full 3x3 tensor (verified convention:
    values are the tensor components themselves, taken about the part's own
    centre of mass, expressed in assembly axes, in kg*cm^2)."""
    ixx, iyy, izz, ixy, iyz, ixz = I6
    return np.array([[ixx, ixy, ixz],
                     [ixy, iyy, iyz],
                     [ixz, iyz, izz]])


def parallel_axis(I_com, m, d):
    """Shift an inertia tensor from the body's COM by offset d."""
    return I_com + m * (float(d @ d) * np.eye(3) - np.outer(d, d))


def combine(parts):
    """parts: list of (mass, com(3), I_about_own_com(3x3)) -> merged triple."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in parts:
        I += parallel_axis(Ii, mi, ci - com)
    return m, com, I


def subtract(whole, part):
    """Remove `part` (mass, com, I_own) from `whole` (mass, com, I_own)."""
    mw, cw, Iw = whole
    mp_, cp, Ip = part
    m = mw - mp_
    com = (mw * cw - mp_ * cp) / m
    # both expressed about `com`, then subtract
    Iw_c = parallel_axis(Iw, mw, cw - com)
    Ip_c = parallel_axis(Ip, mp_, cp - com)
    return m, com, Iw_c - Ip_c


# --------------------------------------------------------------------------
# geometry constants read straight off the CAD (cm, CAD frame)
# --------------------------------------------------------------------------
Y_AXIS_PLANE = -2.75     # every joint axis lies in this horizontal plane
X_HIP = {'F': 15.55, 'R': -5.05}      # pitch-axis x, front / rear
Z_ROLL = {'L': -7.40, 'R': -1.60}     # roll-axis z, left / right
THIGH_LEN = 9.50         # pitch axis -> knee axis
SHANK_LEN = 9.70         # knee axis -> foot-pad centre
FOOT_RADIUS = 1.30       # rubber pad radius  (9.70 + 1.30 = 11.00 to the ground)
LATERAL = 3.98           # roll axis -> leg sagittal plane (outboard)

BASE_ORIGIN = np.array([(X_HIP['F'] + X_HIP['R']) / 2.0,
                        Y_AXIS_PLANE,
                        (Z_ROLL['L'] + Z_ROLL['R']) / 2.0])

# The rear roll brackets were replaced by the front ones on 2026-08-23, so all
# four hips are now the same 'roll축앞다리' sub-assembly and the whole robot is
# symmetric fore-aft about x = 5.25 cm.  That moved the rear pitch axis from
# x = -3.55 to -5.05 cm, i.e. the hip spacing grew from 191 mm to 206 mm.
LEGS = {
    #        fr   lr   hip occurrence      thigh occurrence  calf occurrence   crank        rod
    'FL': ('F', 'L', 'roll축앞다리:1', '왼허벅지:1', '왼종아리발:1', '혼축링크:1', '링크:1'),
    'FR': ('F', 'R', 'roll축앞다리:2', '오른허벅지:2', '오른종아리발:2', '혼축링크:4', '링크:4'),
    'RL': ('R', 'L', 'roll축앞다리:3', '왼허벅지:2', '왼종아리발:2', '혼축링크:2', '링크:2'),
    'RR': ('R', 'R', 'roll축앞다리:4', '오른허벅지:3', '오른종아리발:3', '혼축링크:5', '링크:5'),
}

# visual mesh source document per leg part (raw STL exported in doc frame).
# Both ends now come from the same document; the per-leg occurrence transform
# in mesh_bake is what places and orients each copy.
MESH_SRC = {'F': 'hip_front', 'R': 'hip_front'}
THIGH_MESH = {'L': 'thigh_left', 'R': 'thigh_right'}
CALF_MESH = {'L': 'calf_left', 'R': 'calf_right'}

# Dynamixel horn direction vs URDF joint axis (derived from the servo
# occurrence poses: the part origin sits on the output shaft, offset toward
# the horn face).  +1 => Dynamixel CCW equals positive URDF rotation.
DXL_SIGN = {
    'FL': {'hip': +1, 'thigh': +1, 'calf': +1},
    'FR': {'hip': +1, 'thigh': -1, 'calf': -1},
    'RL': {'hip': -1, 'thigh': +1, 'calf': +1},
    'RR': {'hip': -1, 'thigh': -1, 'calf': -1},
}

# The printed thigh shell.  허벅지.ipt and 허벅지_MIR.ipt are the same solid
# (both 45.1692 cm^3) and both now carry the iProperties mass override for the
# real 3D-printed part, so this guard is a no-op against the current CAD.  It
# is kept because the override lives in the part file and is easy to lose: if
# either side ever reports the solid-ABS figure of 47.879 g again, it is pulled
# back to the real mass and its inertia scaled by the same factor (the centre
# of mass is unchanged because the geometry is identical).
THIGH_SHELL_MASS = 0.022          # kg, per side
THIGH_SHELL_FILES = ('허벅지.ipt', '허벅지_MIR.ipt')


# --------------------------------------------------------------------------
# Weighed mass
#
# The assembled robot reads 960 g on the scale while the CAD adds up to
# 912.6 g.  The missing ~47 g is the wiring harness, fasteners and adhesive
# that the model does not carry.  It is split evenly between the thighs and
# the shanks -- the servo cables run down both -- so one eighth lands on each
# of the four thigh and four calf links.
#
# Each link's inertia is scaled by the same factor as its mass, which treats
# the extra as spread through the link the way its modelled material already
# is.  Adding it as a point mass at the centroid instead would leave the
# inertia untouched and understate the swing inertia of the legs.
# --------------------------------------------------------------------------
MEASURED_TOTAL_MASS = 0.960              # kg, weighed on the assembled robot
MASS_MAKEUP_LINKS = ('thigh', 'calf')    # which links absorb the difference


# --------------------------------------------------------------------------
# Measured fore-aft weight split
#
# Standing at the 140 mm pose the feet sit directly under their own hips, so
# the support midpoint is x = 0 and the front/rear split reads the centre of
# mass directly.  Two weighings, front pair on a scale and rear pair on a block
# at the same height:
#
#     400 g / 580 g   -> front 40.8 %
#     405 g / 600 g   -> front 40.3 %
#
# so the real centre of mass is at about x = -19.5 mm, while the CAD adds up to
# -10.6 mm.  The 47 g of unmodelled wiring cannot explain it: piled at the very
# back of the body it still only moves the total by 4 mm, so the difference is
# in where the assembly puts the body's own contents.
#
# It is applied here as an explicit offset on the base link rather than by
# editing the CAD, so it stays visible and is trivially reverted.  The base
# carries 455.6 g of the 960 g total, so moving the whole robot by -8.9 mm
# means moving the base link's own centre of mass by -8.9 * 960 / 455.6.
#
# The inertia tensor is stated about the link's own centre of mass, so it is
# unchanged by relocating that point - the claim is only that the body's mass
# sits slightly further back than the CAD says, not that it is shaped
# differently.
#
# Set MEASURED_FRONT_FRACTION to None to fall back to the raw CAD figure.
# --------------------------------------------------------------------------
MEASURED_FRONT_FRACTION = 0.4055         # mean of the two weighings


# --------------------------------------------------------------------------
# Actuator: Dynamixel XL330-M288-T, from Rhoban's BAM identification
#
# https://github.com/Rhoban/bam  -- bam/params/xl330/m6.json
#
# BAM ("Better Actuator Models", ICRA 2025) fits a DC-motor + friction model to
# recorded pendulum trajectories. Six friction models m1..m6 are published.
# This package uses m6 and its jointly identified kt/R throughout. The exact
# BAM budget needs external joint torque, which Isaac Lab's actuator API does
# not expose, so XL330BamActuator implements a conservative loaded-motor
# approximation. Do not mix kt/R from one model with another model's friction
# terms: they are fitted together.
#
# The servo is voltage-controlled by its own firmware P controller:
#
#     duty = clip(error_gain * kp_firmware * (q_target - q), -1, 1)
#     tau  = kt * (duty * vin) / R  -  kt^2 * dq / R
#
# Isaac Lab's DCMotor is algebraically identical to this.  It computes
# tau = Kp*dq_err + Kd*(-dq) and clips it to tau_stall*(±1 - dq/dq_max), and
# clipping A*x to [-A, A] before subtracting B is the same as clipping A*x - B
# to [-A-B, A-B].  So the mapping below is exact, not an approximation:
#
#     stiffness        = kt * vin * error_gain * kp_firmware / R
#     damping          = kt^2 / R                       (back-EMF)
#     saturation_effort= kt * vin / R                   (stall torque)
#     velocity_limit   = vin / kt                       (no-load speed)
#     effort_limit     = kt * firmware_current_limit
#     armature         = BAM armature
#     friction         = BAM friction_base   [N*m, Isaac Sim >= 5.0]
#     viscous_friction = BAM friction_viscous
# --------------------------------------------------------------------------
# m6 adds what m1 cannot say: the loss through the gearbox grows with the
# torque passing through it.  On a 288:1 reduction that is the difference
# between a knee that lifts the body and one that does not - at the 1.75 A
# current limit m1 leaves 0.664 N*m at the joint and m6 leaves 0.422.  Measured
# on the robot, the rear knees hit the current limit and still could not stand
# up out of a crouch, which m1 says is comfortable and m6 puts on the edge.
#
# kt and R are refit per model, so they must be taken from the same file as the
# friction terms - m6's lower R is what pays for the extra loss.
BAM_XL330_M6 = {
    'kt': 0.36601349688984386,
    'R': 2.8113923539223227,
    'armature': 0.0018077432831600838,
    'q_offset': 0.0271132870444849,
    # constant part, lives on the PhysX joint
    'friction_base': 0.004771183165566,
    'friction_viscous': 0.005359668274599504,
    # load-dependent part, applied by XL330BamActuator
    'load_friction_motor': 0.2667860954283698,
    'load_friction_external': 8.515871897059342e-06,
    'friction_stribeck': 0.004676345799486616,
    'load_friction_external_stribeck': 0.08077928978935671,
    'load_friction_motor_stribeck': 1.0722918395099123e-05,
    'load_friction_external_quad': 0.004902565732332559,
    'load_friction_motor_quad': 0.009972471242139415,
    'dtheta_stribeck': 2.890372094130307,
    'alpha': 8.683259907618984,
}

# kept for reference: the Coulomb-only fit this package used before
BAM_XL330_M1 = {
    'kt': 0.3866957639281311,          # N*m/A, at the output shaft
    'R': 4.017129222290597,            # Ohm
    'armature': 0.0018141672784458866,  # kg*m^2, reflected to the joint
    'friction_base': 0.01283272036582759,     # N*m, Coulomb
    'friction_viscous': 0.003661672282824312,  # N*m*s/rad
    'q_offset': 0.030871000250528358,
}

# duty cycle produced per (firmware kp * radian of error).
# = (encoder counts per rev / 2pi) / (kp divisor * PWM limit), from BAM's
# XL330Actuator: 4096/(2*pi) / (256 * 885)
XL330_ERROR_GAIN = (4096.0 / (2.0 * np.pi)) / (256.0 * 885.0)
XL330_CURRENT_LIMIT = 1.75         # A, XL330 firmware current limit (Rhoban Microban constants.py)

SUPPLY_VOLTAGE = 7.4               # V, 2S LiPo wired straight to the servos
FIRMWARE_KP = 200                  # XL330 Position P Gain register
# Measured on the robot, closed loop, standing and then walking:
#   kp 400  a 7.5 Hz limit cycle in all three gyro axes - 99 % of the yaw
#           rate power in one band - and 90 degrees of yaw drift in 6 s.
#           1431 mA of it was the servos fighting each other.
#   kp 125  no oscillation, because the servos stopped following: 18 deg
#           of mean tracking error, 47 deg at worst, and the whole robot
#           frozen at the gyro noise floor. Quiet, and not obeying.
#   kp 200  a real gait - 8.5 deg of joint motion, 8.1 deg mean tracking
#           error, 1.1 deg of yaw drift, and 7 % in the 7.5 Hz band.
#           Three consecutive 10 s runs at 0.5 m/s, none of them fell.
# Rhoban run their own RL policy on the same servo at 125 (Rhoban Microban
# constants.py, KP_RL), but they are balancing a biped on one foot at a
# time; a quadruped keeps four feet down and can afford the stiffness.
# DAMPING_SCALE stays where it is: damping is kt^2/R, which does not
# depend on kp, so halving the stiffness raises the damping ratio by
# sqrt(2) on its own - the direction the measurements already show
# (zeta 0.37 at kp 400 against 0.50 at kp 125).
# An earlier static test picked 600: holding a fixed target, 400 let the knees
# sag about 9 degrees under load and stalled them on a lift, while 600 held the
# 170 mm pose to within 3 degrees at under 50 mA per joint.
#
# A closed-loop policy changes that measurement's meaning. It rewrites every
# target at 50 Hz, so steady sag is something it corrects rather than something
# it suffers, and on the robot it held 170 mm for thirty seconds at gains as
# low as 100. What 600 buys instead is a resonance: a swept-sine on the
# standing robot gives a gain of 1.50 at 1.5 Hz at kp=600 against 1.23 at
# kp=400, so the higher gain amplifies exactly the band the shimmer lives in.
# 400 is the compromise - enough bandwidth that commands are actually executed
# (gain 1.11 at 1 Hz, against 0.52 at kp=125, where the servo simply ignores
# the policy) without the peak.
#
# The stall-on-a-lift finding still stands, and still applies to standing up
# from folded, which is a much larger load than standing still.

# Measured damping correction.
#
# The same swept sine, run on the robot and then identically in simulation:
#
#             hardware            simulation
#   kp 400    fn 1.20  zeta 0.37  fn 1.85  zeta 0.92
#   kp 600    fn 1.40  zeta 0.34  fn 1.90  zeta 0.52
#
# The simulated robot is better damped than the real one at both gains, and the
# gap is what the standing policy's 4 Hz shimmer comes out of: it was trained
# against a machine that settles and deployed onto one that rings.
#
# The cause is almost certainly not the motor. `damping` here is the back-EMF
# term kt^2/R, which the real motor has too. What the real robot has *extra* is
# compliance the URDF does not model - printed brackets, bearing play, the
# pushrod - which adds an underdamped mode behind every joint. Isaac Lab has no
# convenient series spring, so the effect is absorbed into the joint damping
# instead. That makes this an empirical fit, not a physical measurement, which
# is why it is a named scale factor sitting next to the physics rather than an
# edit to the formula.
#
# zeta scales with damping at fixed stiffness, so matching 0.92 -> 0.37 is a
# factor of 0.402. Set to 1.0 to get the physical back-EMF value back.
DAMPING_SCALE = 0.402


def actuator_block():
    b = BAM_XL330_M6
    kt, R, vin = b['kt'], b['R'], SUPPLY_VOLTAGE
    stall = kt * vin / R
    motor_limit = min(kt * XL330_CURRENT_LIMIT, stall)
    # Mirror XL330BamActuator's conservative loaded, zero-speed approximation.
    provisional = motor_limit * (1.0 - b['load_friction_motor'])
    stribeck_loss = (
        b['friction_stribeck']
        + b['load_friction_external_stribeck'] * provisional
        + b['load_friction_external_quad'] * provisional * provisional
    )
    net_effort_at_current_limit = max(
        0.0, provisional - stribeck_loss - b['friction_base']
    )
    return {
        'model': 'Dynamixel XL330-M288-T',
        'source': "Rhoban BAM, bam/params/xl330/m6.json (load-dependent friction)",
        # --- raw identified parameters, do not edit
        'bam_model': 'm6',
        'kt': kt,
        'R': R,
        'error_gain': XL330_ERROR_GAIN,
        'friction_base': b['friction_base'],
        'friction_viscous': b['friction_viscous'],
        # --- your hardware choices
        'supply_voltage': vin,
        'firmware_kp': FIRMWARE_KP,
        'firmware_current_limit': XL330_CURRENT_LIMIT,
        # --- derived, fed straight into Isaac Lab's DCMotorCfg
        'stiffness': kt * vin * XL330_ERROR_GAIN * FIRMWARE_KP / R,
        'damping': kt * kt / R * DAMPING_SCALE,
        'damping_backemf': kt * kt / R,
        'damping_scale': DAMPING_SCALE,
        'saturation_effort': stall,
        'effort_limit': motor_limit,
        'velocity_limit': vin / kt,
        'armature': b['armature'],
        'friction': b['friction_base'],
        'dynamic_friction': b['friction_base'],
        'viscous_friction': b['friction_viscous'],
        # --- load-dependent terms, consumed by XL330BamActuator
        'load_friction_motor': b['load_friction_motor'],
        'friction_stribeck': b['friction_stribeck'],
        'load_friction_external_stribeck': b['load_friction_external_stribeck'],
        'load_friction_external_quad': b['load_friction_external_quad'],
        'dtheta_stribeck': b['dtheta_stribeck'],
        'alpha': b['alpha'],
        'net_effort_at_current_limit': float(net_effort_at_current_limit),
        # --- reference points
        'datasheet_stall_torque_5v': 0.52,
        'datasheet_no_load_speed_5v': 8.48,
        'note': 'the gearbox eats load_friction_motor (27%) of whatever the motor '
                'produces, so the torque that reaches the joint at the current limit is '
                'net_effort_at_current_limit, not effort_limit. '
                'stiffness, stall torque, and no-load speed scale with supply_voltage; '
                'the current-limited effort does not. stiffness also scales with firmware_kp. '
                'Rhoban Microban uses kp=125 for RL walking '
                'on a biped; this quadruped carries static load on all four legs, where '
                'kp=125 would let the knees sag ~36 deg. Hardware load sweeps first '
                'selected 600; firmware_kp is now 200, which is what the '
                'robot runs and what the exported hardware map carries.',
    }


def link_origin_cad(leg):
    """Frame origins of the four links of one leg, in CAD cm."""
    fr, lr = LEGS[leg][0], LEGS[leg][1]
    out = -1.0 if lr == 'L' else +1.0          # CAD +Z is the robot's right
    z_leg = Z_ROLL[lr] + out * LATERAL
    hip = np.array([X_HIP[fr], Y_AXIS_PLANE, Z_ROLL[lr]])
    thigh = np.array([X_HIP[fr], Y_AXIS_PLANE, z_leg])
    calf = np.array([X_HIP[fr] - THIGH_LEN, Y_AXIS_PLANE, z_leg])
    foot = np.array([X_HIP[fr] - THIGH_LEN, Y_AXIS_PLANE - SHANK_LEN, z_leg])
    return {'hip': hip, 'thigh': thigh, 'calf': calf, 'foot': foot}


def to_urdf_vec(v_cad):
    return M @ np.asarray(v_cad, dtype=float) * CM


def inertial_block(m, com_cad, I_cad, origin_cad):
    """URDF <inertial> for a body whose merged properties are given in CAD."""
    com_link = M @ (com_cad - origin_cad) * CM
    I = (M @ I_cad @ M.T) * INERTIA
    return {
        'mass': float(m),
        'com': [float(x) for x in com_link],
        'inertia': {
            'ixx': float(I[0, 0]), 'iyy': float(I[1, 1]), 'izz': float(I[2, 2]),
            'ixy': float(I[0, 1]), 'ixz': float(I[0, 2]), 'iyz': float(I[1, 2]),
        },
    }


def box_from_bbox(lo_cad, hi_cad, origin_cad):
    lo = M @ (lo_cad - origin_cad) * CM
    hi = M @ (hi_cad - origin_cad) * CM
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    return {'size': [float(x) for x in (hi - lo)],
            'origin': [float(x) for x in (lo + hi) / 2.0]}


# --------------------------------------------------------------------------
# standing pose solver
# --------------------------------------------------------------------------
def standing_pose(target_base_height, l1=THIGH_LEN * CM, l2=SHANK_LEN * CM,
                  r_foot=FOOT_RADIUS * CM):
    """Symmetric knee-backward crouch with the foot directly under the hip.

    At q = 0 the thigh points backwards (-X) and the shank points down (-Z).
    Rotating about +Y by q swings the leg backwards/up.
    """
    z_foot = -(target_base_height - r_foot)
    lo, hi = 0.0, np.pi / 2 * 0.98
    for _ in range(200):                       # bisect on the thigh sweep beta
        beta = 0.5 * (lo + hi)
        s = (l1 / l2) * np.sin(beta)
        if s > 1.0:
            hi = beta
            continue
        gamma = np.arcsin(s)
        z = -l1 * np.cos(beta) - l2 * np.cos(gamma)
        if z < z_foot:                          # too low -> bend more
            lo = beta
        else:
            hi = beta
    q_thigh = -(np.pi / 2 - beta)
    q_calf = -gamma - q_thigh
    return float(q_thigh), float(q_calf), float(-z + r_foot)


def link_frames(links, joints, q_hip, q_thigh, q_calf):
    """(R, p) of every link in the base frame at the given joint angles."""
    def Ry(a):
        return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])

    def Rx(a):
        return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])

    F = {'base': (np.eye(3), np.zeros(3))}
    for leg in LEGS:
        R0, p0 = F['base']
        ph = p0 + R0 @ np.array(joints['%s_hip_joint' % leg]['origin'], dtype=float)
        Rh = R0 @ Rx(q_hip)
        F['%s_hip' % leg] = (Rh, ph)
        pt = ph + Rh @ np.array(joints['%s_thigh_joint' % leg]['origin'], dtype=float)
        Rt = Rh @ Ry(q_thigh)
        F['%s_thigh' % leg] = (Rt, pt)
        pc = pt + Rt @ np.array(joints['%s_calf_joint' % leg]['origin'], dtype=float)
        Rc = Rt @ Ry(q_calf)
        F['%s_calf' % leg] = (Rc, pc)
        pf = pc + Rc @ np.array(joints['%s_foot_joint' % leg]['origin'], dtype=float)
        F['%s_foot' % leg] = (Rc, pf)
    return F


def total_com(links, joints, q_hip, q_thigh, q_calf):
    """Centre of mass of the whole robot in the base frame."""
    F = link_frames(links, joints, q_hip, q_thigh, q_calf)
    m, s = 0.0, np.zeros(3)
    for name, L in links.items():
        R, p = F[name]
        s += L['mass'] * (p + R @ np.array(L['com'], dtype=float))
        m += L['mass']
    return s / m


def balanced_pose(links, joints, target_height):
    """Crouch whose support polygon is centred on the centre of mass.

    The symmetric ``stand`` pose puts each foot directly under its own hip, so
    the support midpoint is x = 0 while the centre of mass is behind it - the
    rear legs then carry more, which is exactly what the scale showed (40.5 %
    front) and what made the rear knees stall first.

    Sweeping the legs back until the support midpoint sits on the centre of
    mass evens the load out and, because the foot moves closer to under the
    knee, drops the knee torque as well.
    """
    def resid(x):
        t, c = x
        com = total_com(links, joints, 0.0, t, c)
        F = link_frames(links, joints, 0.0, t, c)
        foot = F['FL_foot'][1]
        delta = foot[0] - joints['FL_hip_joint']['origin'][0]   # same for all four legs
        return np.array([(-foot[2] + FOOT_RADIUS * CM) - target_height, delta - com[0]])

    x = np.array([-0.7109, -0.1254])
    for _ in range(200):
        f = resid(x)
        if np.max(np.abs(f)) < 1e-13:
            break
        Jm = np.zeros((2, 2))
        e = 1e-7
        for k in range(2):
            d = x.copy()
            d[k] += e
            Jm[:, k] = (resid(d) - f) / e
        x = x - 0.7 * np.linalg.solve(Jm, f)
    t, c = float(x[0]), float(x[1])
    F = link_frames(links, joints, 0.0, t, c)
    delta = float(F['FL_foot'][1][0] - joints['FL_hip_joint']['origin'][0])
    return t, c, delta


def forward_kinematics(q_hip, q_thigh, q_calf, leg):
    """Foot-centre position in the base frame (m), for verification."""
    fr, lr = LEGS[leg][0], LEGS[leg][1]
    o = link_origin_cad(leg)
    p_hip = M @ (o['hip'] - BASE_ORIGIN) * CM
    side = +1.0 if lr == 'L' else -1.0
    l1, l2 = THIGH_LEN * CM, SHANK_LEN * CM

    def Ry(a):
        return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])

    def Rx(a):
        return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])

    p_thigh = p_hip + Rx(q_hip) @ np.array([0.0, side * LATERAL * CM, 0.0])
    R1 = Rx(q_hip) @ Ry(q_thigh)
    p_knee = p_thigh + R1 @ np.array([-l1, 0.0, 0.0])
    R2 = Rx(q_hip) @ Ry(q_thigh + q_calf)
    p_foot = p_knee + R2 @ np.array([0.0, 0.0, -l2])
    return p_foot


# Values under `hardware` that are measured against the built robot rather
# than derived from CAD. Regenerating them would silently move the robot: the
# horn offsets are where each servo actually sits on its spline, and the zero
# contract says which layer owns that. CAD knows none of it.
MEASURED_HARDWARE_KEYS = ('offset_tick', 'zero_reference',
                          'expected_homing_offset_tick')


def measured_hardware_overlay(existing_params):
    """Pick the measured calibration out of a previously written YAML.

    Args:
        existing_params: The parsed previous ``robot_params.yaml``, or ``None``
            when there is not one yet.

    Returns:
        A dict of the measured keys that were present. Keys that were absent
        are left out, so a first run derives them and later runs keep them.
    """
    hardware = (existing_params or {}).get('hardware') or {}
    return {key: hardware[key] for key in MEASURED_HARDWARE_KEYS if key in hardware}


def apply_measured_hardware(generated, overlay):
    """Lay the measured calibration back over the freshly derived block.

    Args:
        generated: The ``hardware`` block just built from CAD.
        overlay: The result of :func:`measured_hardware_overlay`.

    Returns:
        A new dict: derived values, with every measured key overridden.
    """
    merged = dict(generated)
    merged.update(overlay)
    return merged


# --------------------------------------------------------------------------
def main():
    # The gravity-compensated stand is calibrated in Isaac Sim rather than
    # derived from CAD. Preserve that overlay when regenerating the CAD-owned
    # portion of robot_params.yaml; tune_stand_pose.py --write updates it.
    out = os.path.join(ROOT, 'params', 'robot_params.yaml')
    calibrated_stand = None
    calibrated_stand_155 = None
    # `offset_tick` is the only record of where each servo horn actually sits on
    # its output spline. CAD cannot know it: it is measured against the built
    # robot, and it changes whenever a horn comes off - a gearbox replacement,
    # for instance. Regenerating it as zeros would silently move every joint on
    # the machine by whatever the real offset was.
    measured_hardware = {}
    if os.path.isfile(out):
        with io.open(out, encoding='utf-8') as existing_file:
            existing_params = yaml.safe_load(existing_file) or {}
        calibrated_stand = existing_params.get('poses', {}).get('stand_compensated')
        calibrated_stand_155 = existing_params.get('poses', {}).get('stand_compensated_155')
        measured_hardware = measured_hardware_overlay(existing_params)

    tree, mp, subs = load()

    params = {}
    params['meta'] = {
        'name': 'micro_dog',
        'description': 'Micro Dog quadruped, self-modelled parts inspired by Rhoban Microban (Dynamixel XL330-M288-T x12)',
        'source_cad': '전체.iam (Autodesk Inventor 2027)',
        'units': 'SI (m, kg, rad)',
        'frame': 'ROS/URDF convention: +X forward, +Y left, +Z up',
        'zero_pose': 'CAD assembly pose = every servo at its centre (2048 ticks / 180 deg); '
                     'thigh points straight back, shank points straight down',
        'knee_linkage': 'parallelogram four-bar (crank 19.0 mm, pushrod 95.0 mm) -> exact 1:1 '
                        'coupling, modelled as a single revolute joint like Go2',
    }

    # ---- links -----------------------------------------------------------
    links = {}

    # base ---------------------------------------------------------------
    b = mp['몸:1']
    m_b, com_b, I_b = b['mass'], np.array(b['com']), tensor(b['I'])
    lo, hi = occ_bbox(tree['몸:1'])
    links['base'] = inertial_block(m_b, com_b, I_b, BASE_ORIGIN)
    links['base']['collision'] = {'type': 'box', **box_from_bbox(lo, hi, BASE_ORIGIN)}
    links['base']['mesh'] = 'base'

    geom = {}
    joints = {}

    for leg, (fr, lr, hip_occ, thigh_occ, calf_occ, crank_occ, rod_occ) in LEGS.items():
        o = link_origin_cad(leg)
        side = +1.0 if lr == 'L' else -1.0

        # ---- hip link
        h = mp[hip_occ]
        links['%s_hip' % leg] = inertial_block(h['mass'], np.array(h['com']),
                                               tensor(h['I']), o['hip'])
        lo, hi = occ_bbox(tree[hip_occ])
        links['%s_hip' % leg]['collision'] = {'type': 'box', **box_from_bbox(lo, hi, o['hip'])}
        links['%s_hip' % leg]['mesh'] = 'hip_%s' % leg.lower()

        # ---- thigh link  (= thigh assembly + knee crank + pushrod)
        # the thigh sub-assembly is rebuilt from its own parts so that the
        # printed shell can be forced to its real mass on both sides
        parts = []
        for s in subs[thigh_occ]:
            m_s, I_s = s['mass'], tensor(s['I'])
            if s['file'] in THIGH_SHELL_FILES and abs(m_s - THIGH_SHELL_MASS) > 1e-9:
                k = THIGH_SHELL_MASS / m_s
                m_s, I_s = THIGH_SHELL_MASS, I_s * k
            parts.append((m_s, np.array(s['com']), I_s))
        blo = np.array([np.inf] * 3)
        bhi = np.array([-np.inf] * 3)
        for occ in (thigh_occ, crank_occ, rod_occ):
            l_, h_ = occ_bbox(tree[occ])
            blo = np.minimum(blo, l_)
            bhi = np.maximum(bhi, h_)
        for occ in (crank_occ, rod_occ):
            r = mp[occ]
            parts.append((r['mass'], np.array(r['com']), tensor(r['I'])))
        m_t, com_t, I_t = combine(parts)
        links['%s_thigh' % leg] = inertial_block(m_t, com_t, I_t, o['thigh'])
        links['%s_thigh' % leg]['collision'] = {'type': 'box', **box_from_bbox(blo, bhi, o['thigh'])}
        links['%s_thigh' % leg]['mesh'] = 'thigh_%s' % ('left' if lr == 'L' else 'right')

        # ---- foot: rubber pad modelled as a disc (axis = CAD Z = leg lateral)
        m_f = float(tree['%s/발:1' % calf_occ]['mass_kg'])
        com_f = o['foot'].copy()
        r_, t_ = FOOT_RADIUS, 0.95
        I_f = np.diag([(m_f / 12.0) * (3 * r_ ** 2 + t_ ** 2),
                       (m_f / 12.0) * (3 * r_ ** 2 + t_ ** 2),
                       0.5 * m_f * r_ ** 2])
        links['%s_foot' % leg] = inertial_block(m_f, com_f, I_f, o['foot'])
        links['%s_foot' % leg]['collision'] = {'type': 'sphere', 'radius': FOOT_RADIUS * CM,
                                               'origin': [0.0, 0.0, 0.0]}
        links['%s_foot' % leg]['mesh'] = None

        # ---- calf link = calf assembly minus the rubber pad
        c = mp[calf_occ]
        m_c, com_c, I_c = subtract((c['mass'], np.array(c['com']), tensor(c['I'])),
                                   (m_f, com_f, I_f))
        links['%s_calf' % leg] = inertial_block(m_c, com_c, I_c, o['calf'])
        lo, hi = occ_bbox(tree[calf_occ])
        lo = lo.copy()
        lo[1] = o['foot'][1] + FOOT_RADIUS     # stop the shank box above the pad
        links['%s_calf' % leg]['collision'] = {'type': 'box', **box_from_bbox(lo, hi, o['calf'])}
        links['%s_calf' % leg]['mesh'] = 'calf_%s' % ('left' if lr == 'L' else 'right')

        # ---- joints
        joints['%s_hip_joint' % leg] = {
            'parent': 'base', 'child': '%s_hip' % leg,
            'origin': [float(x) for x in to_urdf_vec(o['hip'] - BASE_ORIGIN)],
            'axis': [1.0, 0.0, 0.0],
        }
        joints['%s_thigh_joint' % leg] = {
            'parent': '%s_hip' % leg, 'child': '%s_thigh' % leg,
            'origin': [0.0, float(side * LATERAL * CM), 0.0],
            'axis': [0.0, 1.0, 0.0],
        }
        joints['%s_calf_joint' % leg] = {
            'parent': '%s_thigh' % leg, 'child': '%s_calf' % leg,
            'origin': [float(-THIGH_LEN * CM), 0.0, 0.0],
            'axis': [0.0, 1.0, 0.0],
        }
        joints['%s_foot_joint' % leg] = {
            'parent': '%s_calf' % leg, 'child': '%s_foot' % leg,
            'origin': [0.0, 0.0, float(-SHANK_LEN * CM)],
            'axis': None, 'type': 'fixed',
        }

        geom[leg] = {
            'hip_origin_in_base': [float(x) for x in to_urdf_vec(o['hip'] - BASE_ORIGIN)],
            'lateral_offset': float(side * LATERAL * CM),
        }

    # ---- reconcile the model with the weighed mass ------------------------
    cad_total = float(sum(v['mass'] for v in links.values()))
    targets = ['%s_%s' % (leg, kind) for leg in LEGS for kind in MASS_MAKEUP_LINKS]
    makeup = (MEASURED_TOTAL_MASS - cad_total) / len(targets)
    for name in targets:
        L = links[name]
        k = (L['mass'] + makeup) / L['mass']
        L['mass'] = float(L['mass'] + makeup)
        for key in L['inertia']:
            L['inertia'][key] = float(L['inertia'][key] * k)

    # ---- shift the base to match the measured fore-aft split --------------
    q_t, q_c, _h = standing_pose(0.14)
    com_cad = total_com(links, joints, 0.0, q_t, q_c)
    com_shift = 0.0
    if MEASURED_FRONT_FRACTION is not None:
        # feet sit under the hips at this pose, so the support spans the hip
        # axes and the split locates the centre of mass directly
        x_front = joints['FL_hip_joint']['origin'][0]
        x_rear = joints['RL_hip_joint']['origin'][0]
        want_x = x_rear + MEASURED_FRONT_FRACTION * (x_front - x_rear)
        com_shift = float(want_x - com_cad[0])
        links['base']['com'][0] += com_shift * MEASURED_TOTAL_MASS / links['base']['mass']

    params['links'] = links
    params['joints'] = joints

    # ---- kinematic summary ----------------------------------------------
    params['kinematics'] = {
        'thigh_length': THIGH_LEN * CM,
        'shank_length': SHANK_LEN * CM,
        'foot_radius': FOOT_RADIUS * CM,
        'knee_to_ground_at_zero': (SHANK_LEN + FOOT_RADIUS) * CM,
        'hip_spacing_x': (X_HIP['F'] - X_HIP['R']) * CM,
        'hip_spacing_y': abs(Z_ROLL['L'] - Z_ROLL['R']) * CM,
        'lateral_offset': LATERAL * CM,
        'nominal_track_width': (abs(Z_ROLL['L'] - Z_ROLL['R']) + 2 * LATERAL) * CM,
        'max_leg_extension': (THIGH_LEN + SHANK_LEN + FOOT_RADIUS) * CM,
        'total_mass': float(sum(v['mass'] for v in links.values())),
        'cad_mass': cad_total,
        'measured_mass': MEASURED_TOTAL_MASS,
        'mass_makeup_per_link': float(makeup),
        'com_x_cad_at_stand': float(com_cad[0]),
        'com_x_measured_at_stand': float(com_cad[0] + com_shift),
        'com_shift_applied': com_shift,
        'measured_front_fraction': MEASURED_FRONT_FRACTION,
        'com_note': 'front/rear weight split measured on a scale at the 140 mm stand '
                    'pose (400/580 g and 405/600 g); the base link centre of mass is '
                    'offset so the model reproduces it',
        'mass_makeup_note': 'the weighed robot is %.1f g heavier than the CAD; the '
                            'difference is wiring/fasteners and is added evenly to the '
                            'four thigh and four calf links, inertia scaled with mass'
                            % ((MEASURED_TOTAL_MASS - cad_total) * 1000),
        'legs': geom,
    }

    # ---- joint limits ----------------------------------------------------
    # q = 0 is the CAD pose:
    #   hip   0 -> leg plane vertical
    #   thigh 0 -> thigh horizontal, pointing backwards ; -pi/2 -> straight down
    #   calf  0 -> shank straight down (90 deg knee) ; negative folds the knee
    #
    # Hip-roll is asymmetric per side. ID 31 gives the left inward endpoint
    # (-1.55545651891604 rad), while ID 41 gives the right outward endpoint
    # (-0.8605632220038447 rad). Mirroring those measurements gives the other
    # end of each side's range. Hip-pitch endpoints were measured the same way: ID 42 at
    # -1.5278448647340985 rad and ID 32 at +0.859029241215959 rad after the
    # right-leg direction sign is applied. Calf endpoints came from ID 43
    # fully folded at -1.101398206 rad and ID 33 fully extended at
    # +1.207242880 rad.
    params['limits'] = {
        'hip': {
            # Safe intersection retained as the fallback for tools that do not
            # yet understand side-specific limits.
            'lower': -0.8605632220038447,
            'upper': 0.8605632220038447,
            'left': {
                'lower': -1.55545651891604,
                'upper': 0.8605632220038447,
            },
            'right': {
                'lower': -0.8605632220038447,
                'upper': 1.55545651891604,
            },
            'source': 'measured 2026-09-02; ID31 left inward endpoint and ID41 '
                      'right outward endpoint, mirrored by side',
        },
        'thigh': {
            'lower': -1.5278448647340985,
            'upper': 0.859029241215959,
            'source': 'measured 2026-09-02; ID42 lower endpoint, ID32 upper endpoint '
                      'after direction signs',
        },
        'calf': {
            'lower': -1.101398206,
            'upper': 1.207242880,
            'source': 'measured 2026-09-02; ID43 fully folded, ID33 fully extended',
        },
    }
    calf_margin = math.radians(5.0)
    params['command_limits'] = {
        'hip': {
            'lower': -0.8605632220038447 + calf_margin,
            'upper': 0.8605632220038447 - calf_margin,
            'left': {
                'lower': -1.55545651891604 + calf_margin,
                'upper': 0.8605632220038447 - calf_margin,
            },
            'right': {
                'lower': -0.8605632220038447 + calf_margin,
                'upper': 1.55545651891604 - calf_margin,
            },
            'safety_margin_rad': calf_margin,
            'safety_margin_deg': 5.0,
            'source': 'measured mirrored hip-roll endpoints, inset 5 degrees at both ends',
        },
        'thigh': {
            'lower': params['limits']['thigh']['lower'] + calf_margin,
            'upper': params['limits']['thigh']['upper'] - calf_margin,
            'safety_margin_rad': calf_margin,
            'safety_margin_deg': 5.0,
            'source': 'measured hip-pitch travel endpoints, inset 5 degrees at both ends',
        },
        'calf': {
            'lower': params['limits']['calf']['lower'] + calf_margin,
            'upper': params['limits']['calf']['upper'] - calf_margin,
            'safety_margin_rad': calf_margin,
            'safety_margin_deg': 5.0,
            'source': 'measured calf travel endpoints, inset 5 degrees at both ends',
        },
    }

    # ---- actuator --------------------------------------------------------
    params['actuator'] = actuator_block()

    # ---- default poses ---------------------------------------------------
    # 170 mm rather than 140: standing taller shortens the knee's lever arm, so
    # the static knee torque drops from 0.140 to 0.104 N-m even though the robot
    # is higher.  83% of full leg extension, both joints well inside their limits.
    target_h = 0.17
    q_t, q_c, h = standing_pose(target_h)
    q_tb, q_cb, delta_b = balanced_pose(links, joints, target_h)
    # Locomotion crouch requested after the 170 mm policy visibly dragged its
    # feet.  Keep this as a separate pose: changing `stand_balanced` would also
    # move every recovery and hardware setup task that relies on the 170 mm
    # design stand.
    locomotion_h = 0.155
    q_tl, q_cl, delta_l = balanced_pose(links, joints, locomotion_h)
    err = forward_kinematics(0.0, q_t, q_c, 'FL')
    params['poses'] = {
        'zero': {'note': 'CAD assembly pose, every servo at 2048 ticks',
                 'hip': 0.0, 'thigh': 0.0, 'calf': 0.0,
                 'base_height': (SHANK_LEN + FOOT_RADIUS) * CM},
        'stand': {'note': 'symmetric crouch, feet directly under the hips',
                  'hip': 0.0, 'thigh': round(q_t, 4), 'calf': round(q_c, 4),
                  'base_height': round(h, 4),
                  'fk_check_foot_in_base': [round(float(x), 5) for x in err]},
        'stand_balanced': {
            'note': 'legs swept back until the support midpoint sits on the centre of '
                    'mass, so the front and rear pairs carry the same load; this is '
                    'the pose the gravity compensation and the RL default are built on',
            'hip': 0.0, 'thigh': round(q_tb, 4), 'calf': round(q_cb, 4),
            'base_height': target_h,
            'foot_offset_from_hip': round(delta_b, 5),
            'load_front_fraction': 0.5,
        },
        'stand_balanced_155': {
            'note': 'task-local locomotion crouch; support midpoint centred on the '
                    'measured centre of mass at 155 mm base height',
            'hip': 0.0, 'thigh': round(q_tl, 4), 'calf': round(q_cl, 4),
            'base_height': locomotion_h,
            'foot_offset_from_hip': round(delta_l, 5),
            'load_front_fraction': 0.5,
        },
    }
    if calibrated_stand is not None:
        params['poses']['stand_compensated'] = calibrated_stand
    if calibrated_stand_155 is not None:
        params['poses']['stand_compensated_155'] = calibrated_stand_155

    # ---- hardware mapping ------------------------------------------------
    params['hardware'] = apply_measured_hardware({
        'protocol': 'Dynamixel Protocol 2.0',
        'tick_per_rev': 4096,
        'centre_tick': 2048,
        'sign': DXL_SIGN,
        'offset_tick': {leg: {j: 0 for j in ('hip', 'thigh', 'calf')}
                        for leg in LEGS},
        'zero_reference': {'owner': 'software_offset_tick',
                           'verify_on_start': True},
        'expected_homing_offset_tick': {
            leg: {j: 0 for j in ('hip', 'thigh', 'calf')} for leg in LEGS},
        'note': 'tick = centre_tick + sign * (q_urdf / (2*pi)) * 4096 + offset_tick. '
                'The knee four-bar is an exact parallelogram, so the knee servo angle '
                'equals the shank angle relative to the thigh, 1:1, no extra coupling term.',
    }, measured_hardware)

    # ---- mesh baking table ----------------------------------------------
    bake = {}
    R, p = occ_pose(tree['몸:1'])
    bake['base'] = [{'stl': 'body_coarse.stl',
                     'R': [list(map(float, row)) for row in (M @ R)],
                     't': [float(x) for x in (M @ (p - BASE_ORIGIN) * CM)]}]
    for leg, (fr, lr, hip_occ, thigh_occ, calf_occ, crank_occ, rod_occ) in LEGS.items():
        o = link_origin_cad(leg)
        R, p = occ_pose(tree[hip_occ])
        bake['hip_%s' % leg.lower()] = [{'stl': '%s_coarse.stl' % MESH_SRC[fr],
                                         'R': [list(map(float, row)) for row in (M @ R)],
                                         't': [float(x) for x in (M @ (p - o['hip']) * CM)]}]
        key = 'thigh_%s' % ('left' if lr == 'L' else 'right')
        if key not in bake:
            entries = []
            for occ, stl in ((thigh_occ, THIGH_MESH[lr] + '_coarse.stl'),
                             (crank_occ, 'crank_coarse.stl'),
                             (rod_occ, 'rod_coarse.stl')):
                R, p = occ_pose(tree[occ])
                entries.append({'stl': stl,
                                'R': [list(map(float, row)) for row in (M @ R)],
                                't': [float(x) for x in (M @ (p - o['thigh']) * CM)]})
            bake[key] = entries
        key = 'calf_%s' % ('left' if lr == 'L' else 'right')
        if key not in bake:
            R, p = occ_pose(tree[calf_occ])
            bake[key] = [{'stl': '%s_coarse.stl' % CALF_MESH[lr],
                          'R': [list(map(float, row)) for row in (M @ R)],
                          't': [float(x) for x in (M @ (p - o['calf']) * CM)]}]
    params['mesh_bake'] = bake

    with io.open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(params, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    print('wrote %s' % out)

    # ---- console summary -------------------------------------------------
    print('\nCAD mass              %.4f kg' % cad_total)
    print('measured mass         %.4f kg  (+%.2f g spread over %d links, %+.3f g each)'
          % (MEASURED_TOTAL_MASS, (MEASURED_TOTAL_MASS - cad_total) * 1000,
             len(targets), makeup * 1000))
    print('total mass            %.4f kg' % params['kinematics']['total_mass'])
    if MEASURED_FRONT_FRACTION is not None:
        print('COM at stand          %+.2f mm cad -> %+.2f mm measured  (base shifted %+.2f mm)'
              % (com_cad[0] * 1000, (com_cad[0] + com_shift) * 1000,
                 com_shift * MEASURED_TOTAL_MASS / links['base']['mass'] * 1000))
    print('thigh / shank         %.1f / %.1f mm' % (THIGH_LEN * 10, SHANK_LEN * 10))
    print('knee -> ground        %.1f mm' % ((SHANK_LEN + FOOT_RADIUS) * 10))
    print('hip spacing x / y     %.1f / %.1f mm' % (params['kinematics']['hip_spacing_x'] * 1000,
                                                    params['kinematics']['hip_spacing_y'] * 1000))
    print('track width           %.1f mm' % (params['kinematics']['nominal_track_width'] * 1000))
    print('stand pose            thigh %.4f  calf %.4f  -> base height %.4f m'
          % (q_t, q_c, h))
    print('balanced stand        thigh %.4f  calf %.4f  -> feet %+.1f mm behind the hips'
          % (q_tb, q_cb, delta_b * 1000))
    print('  FK foot in base     %s' % np.array2string(err, precision=5))
    print()
    for name in sorted(links):
        L = links[name]
        print('  %-10s m=%7.4f kg  com=%s' % (name, L['mass'],
              np.array2string(np.array(L['com']), precision=4, suppress_small=True)))


if __name__ == '__main__':
    main()
