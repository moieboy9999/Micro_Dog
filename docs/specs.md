# Micro Dog — design specifications

This document derives the URDF from the Inventor CAD. All dimensions come from the assembly. Total
mass, servo gain and supply voltage were measured on the robot.

## 1. Frames

The CAD assembly frame is +X forward, +Y up, +Z right. The URDF uses the ROS convention: +X
forward, +Y left, +Z up. One fixed rotation maps between them:

```
p_urdf = M · p_cad        M = [[1, 0, 0],
                               [0, 0,-1],
                               [0, 1, 0]]
```

The `base` origin is the centre of the four hip-roll axes, the same convention as Go2.

## 2. Dimensions (from CAD)

| Item | Value |
|---|---|
| Hip-roll axis spacing, fore-aft | 206.0 mm |
| Hip-roll axis spacing, across | 58.0 mm |
| Roll axis → leg plane (lateral offset) | 39.8 mm |
| Thigh (pitch axis → knee axis) | 95.0 mm |
| Shank (knee axis → foot pad centre) | 97.0 mm |
| Foot radius (TPU-95A foot) | 13.0 mm |
| Knee axis → ground contact | 110.0 mm |
| Track width (left–right foot spacing) | 137.6 mm |
| Maximum leg extension | 205.0 mm |

The roll, pitch and knee axes all lie in one horizontal plane, and the roll and pitch axes
intersect. The only `hip_joint → thigh_joint` offset is therefore the 39.8 mm lateral one.

### Mass budget

| Link | Mass | Contents |
|---|---|---|
| `base` | 455.6 g | frame 220.6 g; system 235.0 g (2 batteries, Pi + HAT, BMS, charger, fan, switch); 4 roll servos |
| `*_hip` ×4 | 33.6 g | roll horn frame, pitch servo, bearing |
| `*_thigh` ×4 | 66.8 g | thigh shell 22.0, XL330 22.0, large bearing 2.0, crank 2.6, pushrod 12.2, **measured makeup 5.9** |
| `*_calf` ×4 | 23.8 g | shank + foot assembly minus the foot tip, **measured makeup 5.9** |
| `*_foot` ×4 | 2.0 g | TPU-95A foot tip |
| **Total** | **960.0 g** | CAD 912.6 g + makeup 47.4 g |

**Measured makeup.** The assembled robot weighs 960 g against a CAD total of 912.6 g.
- The 47.4 g the CAD does not carry is wiring, screws and glue.
- The servo cables run down the legs, so it is split evenly over the four thighs and four calves:
  +5.925 g per link.
- Each link's inertia tensor is scaled by the same ratio.
- A point mass at the COM would leave the inertia unchanged and under-state leg swing inertia.

**Material density.** CAD part masses use Inventor's ABS density (1.06 g/cm³), except where a
mass is overridden (the thigh shell is fixed at 22.0 g). The printed frame is PLA
(about 1.24 g/cm³). The weighed total of 960 g is exact. The split between links still assumes ABS
density for the printed parts.

Inertia tensors come from Inventor `MassProperties`: about the COM, in assembly axes, converted
from kg·cm² to kg·m². The sign and order convention was checked against the principal moments.

## 3. Knee: four-bar → one revolute joint

The knee servo sits in the thigh. It drives crank → pushrod → shank crank arm → shank.

| Link | Length |
|---|---|
| Thigh (ground link) | 95.0 mm |
| Crank | 19.0 mm |
| Pushrod | 95.0 mm |
| Shank crank arm | 19.0 mm |

This is an exact parallelogram, so the crank and the shank arm stay parallel. The shank angle
relative to the thigh equals the knee servo angle 1:1, with no coupling term. A single revolute
knee is therefore an exact equivalent model, not an approximation.

The pushrod (12.2 g) and crank (2.6 g) are merged into the thigh link. In a parallelogram the
pushrod always moves parallel to the thigh, so this is physically sound.

## 4. Joint zero and servo mapping

`q = 0` is the CAD assembly pose. All 12 servos are at 2048 ticks (180°). The thigh points
straight back and the shank straight down.

```
tick = 2048 + sign · q · 4096/(2π) + offset_tick
q    = sign · (tick − 2048 − offset_tick) · 2π/4096
```

| Joint | FL | FR | RL | RR |
|---|---|---|---|---|
| hip | +1 | +1 | −1 | −1 |
| thigh | +1 | −1 | +1 | −1 |
| calf | +1 | −1 | +1 | −1 |

Servo IDs are leg-major:

| | FL | FR | RL | RR |
|---|---|---|---|---|
| hip roll | 11 | 21 | 31 | 41 |
| thigh pitch | 12 | 22 | 32 | 42 |
| knee pitch | 13 | 23 | 33 | 43 |

### Poses

| Pose | hip | thigh | calf | base height |
|---|---|---|---|---|
| `zero` (assembly) | 0 | 0 | 0 | 0.110 m |
| `stand` (kinematic target) | 0 | −0.9500 | +0.3440 | 0.170 m |
| `stand_balanced` (COM over support centre) | 0 | −0.8380 | +0.3658 | 0.170 m |
| `stand_compensated` (commanded, under gravity) | +0.0084 | −0.8149 | +0.5002 | 0.168 m (target 0.170) |

`stand_compensated` is the command whose steady state under gravity approximates `stand_balanced`.
The XL330 position loop has finite stiffness, so the commanded angles have to lead the target. All
poses, including 0.155 m variants, are in `params/robot_params.yaml → poses`, which is the source
of truth for these values.

## 5. Actuator constants

`params/robot_params.yaml → actuator` holds these constants for the XL330-M288-T:
- effort limit and velocity limit;
- torque constant, winding resistance and armature;
- friction.

The friction and motor constants come from Rhoban BAM (`xl330/m6.json`; see ATTRIBUTION.md). The
effort and velocity limits are evaluated at this robot's supply voltage.
