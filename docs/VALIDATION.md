# Validation record

## Verified here

The standalone browser dynamics module is exercised with Node's built-in test runner. The tests cover:

1. Zero-force edge cases in the tire formula
2. Odd symmetry and `|Fy| ≤ mu Fz` across the slip range
3. Combined longitudinal/lateral friction-circle constraint
4. Steering's return-to-center stability
5. Speed-dependent steering sensitivity
6. Bounded steering correction under sideslip
7. Finite states across drive/brake/reverse and changing friction
8. Straight-line acceleration
9. Reverse behavior at parking speed
10. Zero-input rest-state stability

Command: `npm test` (Node 20+). CI: `.github/workflows/ci.yml`.

## Still unverified

- Unity C# compilation and NUnit EditMode tests (Unity Editor not available in this environment)
- PhysX WheelCollider chassis tuning, visual wheel mesh orientation, ground collision
- Runtime GC allocations in Unity player on iOS
- A15/A16 CPU/GPU frametime, throttling and sustained thermals
- Real tire coefficient fitting or measured vehicle turning radius

## Suggested Sprint 2 acceptance tests

- In Unity profiler, compare `FixedUpdate` GC Alloc and `Physics.Simulate` CPU time for 30 minutes.
- Test stationary spring equilibrium and 5/10 km/h fixed-steer loops.
- Run `WheelCollider.ConfigureVehicleSubsteps` at low speed with 1 vs 3 substeps, evaluate impulse jitter.
- Record iPhone's `ProcessInfo.thermalState` and render/physics frame times while toggling mirror cameras and rendering scale.
- Reject silent physics step changes as a thermal fix: keep physical timestep stable and degrade visual features first.
