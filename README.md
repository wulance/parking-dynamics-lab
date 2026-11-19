# Parking Dynamics Lab

**Mobile-first vehicle dynamics prototype · 可运行的手机驾驶物理原型**

A small, inspectable engineering demo for an iOS parking simulator. Includes a **playable browser demo** (no install), a **standalone C# vehicle model for Unity**, and **automated tests**. It is an honest technical artifact rather than a claim that a production-quality driving simulator has already been built.

## What works now

- **Top-down driveable sedan** with steer / throttle / brake / reverse and traced vehicle path
- **Simplified Pacejka Magic Formula** lateral tire force with normal-load dependence
- **Combined-slip friction circle** constraining longitudinal + lateral forces
- **Touch steering controller**: nonlinear mapping, speed-sensitive steering angle and bounded yaw/sideslip counter-steer
- **Low-speed kinematic blending** to prevent division-by-near-zero sideslip jitter in parking scenarios
- **Fixed 50 Hz physics step** with animation decoupled from simulation steps
- **Live telemetry** for yaw rate, sideslip angle, lateral forces and friction utilization
- **C# modules** for Unity 6, with optional `UnityWheelColliderBackend`
- **Node tests** and GitHub Actions CI for the browser/reference physics

## Try the demo

Serve the files locally (ES module imports require HTTP):

```sh
python3 -m http.server 8080
# Open http://localhost:8080
```

Mobile: drag the steering wheel, hold throttle/brake, toggle **D / R**. Desktop: **W** throttle, **S** brake, **A / D** steer, **R** reverse. Toggle stability assist and change road friction to compare responses. The page uses no external assets or npm dependencies.

Run tests:

```sh
npm test
```

## Source layout

```text
index.html                      Browser demo
src/app.js                     Canvas rendering, touch/pointer controls, 50 Hz loop
src/dynamics.js                Runnable planar bicycle + steering + tire math
src/style.css                  Responsive mobile layout
tests/dynamics.test.js         Automated math + vehicle behavior checks
unity/ParkingDynamics/Core/    C# contracts, math and no-allocation telemetry ring
unity/ParkingDynamics/Tires/   Pacejka lateral force and combined-slip cap
unity/ParkingDynamics/Control/ Bounded steering controller
unity/ParkingDynamics/Simulation/
  PlanarBicycleBackend.cs      Pure C# reference backend
  UnityWheelColliderBackend.cs Optional Unity 3D hookup sample
unity/ParkingDynamics/Tests/   NUnit examples (requires Unity Test Framework)
docs/DESIGN.md                 Model assumptions, architecture, limitations
docs/VALIDATION.md             Testing and honest performance claims
```

## Unity integration (not a ready-to-open Unity project)

1. Create a new **Unity 6 URP** project. Copy `unity/ParkingDynamics/` under `Assets/`.
2. Add a Rigidbody, 4 WheelColliders and 4 visual wheel Transforms to a car object.
3. Attach `UnityWheelColliderBackend` and wire its references. Add an external UI input component to populate `InputSteering`, `InputThrottle`, `InputBrake`, and `Reverse`.
4. In Unity Test Framework, put tests into a test assembly if desired. No asmdefs or generated Unity `.meta` are included in this portable source package.
5. Do not apply `PacejkaTireModel` forces on top of active WheelColliders: they already compute friction. The planar model and WheelCollider model are **alternative backends**, not additive.

**Important:** This repository does *not* claim a built iOS application, validated Apple silicon thermal metrics, accurate suspension, or real-world tire calibration. The browser demo is a 2D conceptual simulator; the Unity C# code has not been compiled in Unity in this environment. Read [model limitations](docs/DESIGN.md) and [validation status](docs/VALIDATION.md).

## Next steps

1. Import the C# source into Unity 6 and run EditMode/PlayMode tests.
2. Create a parking lot Unity scene and configure real suspension mass/spring/damper values.
3. Profile 50 Hz physics cost and allocations on A15/A16 iPhones; test thermal behavior under a 30-minute workload.
4. If the target requires validated Pacejka behavior, implement a dedicated raycast/contact backend instead of blending its forces with WheelCollider.

MIT licensed. Created as a starting point for a technical portfolio and iterative engineering work.

## Publish to your GitHub

1. Create a new **empty** repository at `https://github.com/new` (e.g. `parking-dynamics-lab`). Do **not** generate a remote README or .gitignore.
2. Extract this project on your Mac and open Terminal in the extracted folder.
3. Run:

```sh
bash publish-to-github.sh https://github.com/YOUR_USERNAME/parking-dynamics-lab.git
```

4. In GitHub repository **Settings → Pages**, choose **Deploy from a branch**, branch `main`, folder `/ (root)` to publish the HTML demo. The GitHub Pages URL is available after the deployment finishes.

You will need an authenticated Git installation (e.g. GitHub CLI credential helper or SSH). The script does not request or store tokens.
