# Body cover

The body cover is `cad/inventor/몸/커버/몸커버.ipt` ("body cover"). It was added to the design on
2026-09-13, after the 2026-08-23 export that the URDF, STEP, meshes and USD come from. **It is not
part of the simulation model.**

| Property | Value |
|---|---|
| Material | ABS |
| Volume | 40.9 cm³ |
| CAD mass (solid ABS density) | 43.4 g |

- **How much it weighs is not settled.** The CAD value above assumes a fully solid part. A printed
  cover weighs less, depending on infill and wall settings, so weigh your own print.
- **The URDF total of 0.960 kg was weighed before the cover was fitted.** A robot with the cover
  is heavier by the cover's printed mass.
- **To include the cover in simulation,** add its measured mass to the `base` link. It mounts on
  the body, so treat it as part of `base`. Then scale the `base` inertia or recompute it from CAD.
- For gait work at the current fidelity, the model has been used without the cover.
