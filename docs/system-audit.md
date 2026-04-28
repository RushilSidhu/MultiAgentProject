# Current MATLAB System Audit

Legacy source files referenced in this audit are archived in `legacy/matlab/`.
The active implementation in this repository is TypeScript under `src/`.

This audit maps the main system flaws in the MATLAB prototype to replacement
modules in the visual TypeScript revamp. The robot's current planning algorithm
is intentionally excluded because it is expected to change.

## Flaws And Replacement Boundaries

- `HospitalEnv.m` owns geometry, task state, dispatch, coordination, metrics,
  rendering, and UI. Replace it with separate simulation-core, scenario-model,
  coordination, telemetry, and visual-app modules.
- The stated robot observability model is inconsistent: the README says robots
  have no global knowledge, but `RobotAgent` receives the full static map and
  hidden environment holds. Replace this with an explicit policy observation
  contract.
- Environment-level critical-zone reservations and task-space tokens make the
  world silently help the robots. Move coordination into named services whose
  behavior can be enabled, disabled, and compared.
- Tasks are only point targets. Replace them with lifecycle-driven requests that
  include origin, destination, priority, service time, deadline, status, and
  assignment history.
- Nurses are moving obstacles rather than agents with intent. Represent staff
  as agents with routes, dwell behavior, right-of-way rules, and optional carts
  that affect both visuals and physical footprint.
- Collisions, push-out physics, and robot retirement are mixed across agent and
  environment code. Centralize collision queries and event emission in the core.
- Near misses are counted every timestep, which inflates a single close pass.
  Track near-miss episodes with start/end times and closest distance.
- Rendering is coupled to simulation state and can affect benchmark execution.
  Render from immutable snapshots and keep headless runs independent.
- Benchmarks use one seed and mix several strategy changes at once. Use scenario
  ids, seeds, policy ids, event logs, and replay snapshots for reproducibility.
- Geometry and POIs are hard-coded for one 60 x 40 map. Move rooms, obstacles,
  POIs, spawn zones, and task classes into data-driven scenario files.

## Replacement Modules

- `src/sim/engine.ts`: deterministic fixed-step kernel and event log.
- `src/scenarios/hospitalScenario.ts`: data-driven ward layout and task model.
- `src/sim/policies.ts`: clean robot policy observation/action API.
- `src/sim/geometry.ts`: shared collision and distance helpers.
- `src/render/CanvasRenderer.tsx`: visual renderer driven by snapshots.
- `src/App.tsx`: controls, dashboard, seed selection, and replay timeline.
