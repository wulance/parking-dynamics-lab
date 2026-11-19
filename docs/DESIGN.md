# Design notes and explicit physics assumptions

## Motivation

On a touchscreen, drivers have neither steering wheel torque feedback nor a large visual field. This demo keeps tire forces non-linear but adds **bounded input-side assistance**, not teleportation or direct rigidbody yaw damping.

## Core pipeline

`DriverInput` → `TouchSteeringAssist` → `PlanarBicycleBackend` → `PacejkaTireModel` → planar force/moment integration → telemetry. The browser version mirrors these concepts in JS and offers a playable test surface. The Unity sample uses a separate `UnityWheelColliderBackend` that delegates full contact and suspension to PhysX.

## Tire dynamics

Simplified Magic Formula (lateral only):

`Fy = -mu * Fz * sin(C * atan(B*alpha - E*(B*alpha - atan(B*alpha))))`

Default parameters: `B = 9.8`, `C = 1.35`, `E = 0.45`. These values are design examples, **not calibrated tire coefficients**. There is no camber thrust, aligning moment, pressure, temperature, or detailed dynamic load sensitivity. The combined-slip cap enforces `sqrt(Fx² + Fy²) ≤ mu Fz`; it is a friction circle approximation, not full combined-slip Pacejka.

For `|u| < 1.5 m/s`, slip angle measurements are ill-conditioned, so beta estimation is disabled and the planar model blends its yaw response toward the kinematic bicycle limit for speeds below around `2.75 m/s`. That is explicitly a low-speed stability expedient. It should be replaced or calibrated for a high-fidelity tire model.

## Touch steering control

1. Shape normalized touch input with exponent 1.55 to add precision near center.
2. Bound steering angle based on speed.
3. Estimate desired yaw `r_ref = u*tan(delta)/wheelbase`, then cap by `mu*g/max(|u|, 2)`.
4. Correction `delta_assist = clamp(-K_beta*beta - K_yaw*(r-r_ref), ±0.18 rad)`.
5. Fade assist out below ~`3.5 m/s`, including parking maneuvers.
6. Rate-limit steering and the assist signal. No direct edits to rigidbody velocities.

## Suspension and Unity backends

`UnityWheelColliderBackend` configures a single vehicle-level substep profile once, uses a rigidbody with interpolation, controls all four WheelColliders with torques, and updates visual wheels in `LateUpdate`. It does **not** apply additional Pacejka forces because that would double count contact friction.

The planar bicycle simulation is 2D and has no spring/damper contact solver, pitch or roll. A later phase should implement load transfer and suspension tuning with real wheel contact data, not infer that the 2D model already verifies this.

## Allocation and mobile performance strategy

- Pure C# physics modules use structs and fixed-size state, with no per-step collections or strings.
- A preallocated `NoAllocTelemetryBuffer` stores a bounded rolling window without reallocating.
- Avoid `Debug.Log`, LINQ, closures, event subscriptions or CSV serialization inside `FixedUpdate`.
- Unity 3D example resolves `Rigidbody` and Wheels only during initialization; visual transforms are optional.
- Treat zero heap allocations in the hot path as **a design intention pending Unity Profiler measurement**, not a verified claim.
- The browser caps device pixel ratio to 2 and limits physics catch-up steps; it does not measure iOS hardware temperature.

## Non-goals for the prototype

- Photorealistic 3D rendering, full powertrain, torque curves, ABS, gearbox simulation
- True four-corner sprung-mass suspension, transient tire relaxation, accurate load transfer
- Exact iPhone A15/A16 60 FPS thermal validation
- Real-car certification or actual driving instruction
