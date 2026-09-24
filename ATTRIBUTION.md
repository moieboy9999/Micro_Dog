# Attribution and third-party components

The Micro Dog mechanical design was modelled by the author and is released under CC BY-NC-SA 4.0
(see `LICENSE`). That covers:
- every part in `cad/inventor/` except the ROBOTIS XL330 models (`부품/XL,XC-330*.ipt`). That
  includes all printed frames, brackets, links, legs, feet and the body cover, and the envelope
  models of the purchased electronics and bearings, `BMS.ipt` among them;
- the STEP, meshes, URDF and USD derived from it.

The servo horns are the stock horns supplied with the XL330 (ROBOTIS). They are part of the
ROBOTIS servo model, not a separate design.

## Inspiration

**Rhoban Microban** — <https://github.com/Rhoban/microban>
- Micro Dog took its system layout and bill-of-materials idea from Microban: Pi Zero 2 W,
  XL330 servos, a HAT, 2-cell lithium power with BMS and charger.
- No Microban CAD, STL, STEP or source files are included or derived here.
- Microban is licensed CC BY-NC-SA 4.0 (hardware/docs) and GPL-3.0 (software).

## Components used by the robot (not redistributed here)

| Component | Source | License | In this repository |
|---|---|---|---|
| Pollen Robotics RPI Robot HAT | <https://github.com/pollen-robotics/elec_RPI_Robot_HAT> | Apache-2.0 | Not included. `cad/inventor/부품/라파+hat.ipt` is a self-made envelope model of the board, not the PCB design. |
| Raspberry Pi Zero 2 W | Raspberry Pi Ltd. | — | 3D model **not included** (third-party model); see README → Inventor source. |

## Third-party data and models included

| Item | Source | Notes |
|---|---|---|
| Dynamixel XL330-M288-T CAD model (`cad/inventor/부품/XL,XC-330.ipt`, `XL,XC-330_MIR.ipt`); XL330 geometry fused into the STEP and visual meshes | ROBOTIS, public CAD download (<https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/>) | © ROBOTIS. Included for assembly completeness. |
| XL330 actuator friction / motor constants in `params/robot_params.yaml` (`actuator` block) | Rhoban BAM, `bam/params/xl330/m6.json` — <https://github.com/Rhoban/bam> | Apache-2.0 |

Link and joint names follow the Unitree Go2 naming convention. No Unitree assets are included.
