# Hospital Multi-Agent Simulation — Architecture Reference

A fully object-oriented, handle-class MATLAB simulation shell for a 2-D
top-down hospital environment. Designed as a clean boilerplate layer so you
can drop in your own game-theory path-planning algorithms without touching
any simulation infrastructure.

---

## File Map

| File | Role |
|---|---|
| `main.m` | Entry-point. Configuration lives here. |
| `HospitalEnv.m` | 60×40 m ward layout (walls, rooms, POIs), entity registry, renderer, collision, metrics. |
| `RobotAgent.m` | Unicycle-model robot with 2-D Lidar sensor and pluggable controller. |
| `NurseAgent.m` | Waypoint / random walk (plain nurse or nurse+cart); optional semantic `Waypoints` from env. |
| `TaskDispatcher.m` | Normally-distributed task spawner with optional batch arrivals. |

---

## Architecture Diagram

```
main.m
 │
 ├─► HospitalEnv          (handle)
 │     ├─ Obstacles[]     (walls + beds + furniture, N×4 AABB)
 │     ├─ Rooms[]         (floor patches + labels)
 │     ├─ POIs[]          (named task / nurse stops)
 │     ├─ Robots[]        (RobotAgent handles)
 │     ├─ Nurses[]        (NurseAgent handles)
 │     ├─ Tasks[]         (struct: id,x,y,assignedTo,spawnTime,poiName)
 │     ├─ step(dt)        ← calls all agents, collision engine, metrics
 │     └─ render()        ← single-figure, throttled drawnow
 │
 ├─► TaskDispatcher       (handle)
 │     └─ update(dt)      ← samples inter-arrival, calls env.spawnTask()
 │
 ├─► RobotAgent × N      (handle)
 │     ├─ senseEnvironment(obstacles, nurses, otherRobots)
 │     │     └─ returns lidarData struct  {ranges, angles}
 │     ├─ calculateVelocity(tx, ty, lidarData)   ← OVERRIDE THIS
 │     └─ move(v, ω, dt, obstacles, W, H)
 │
 └─► NurseAgent × K      (handle)
       ├─ setWaypoints(Nx2)  ← optional semantic stops
       └─ step(dt, obstacles, W, H)
```

---

## Key Design Decisions

### Handle Classes
All entities inherit from `handle`. Assignments pass **references**, so
`env.Robots(i).X = 5` modifies the robot in place — no value-copy gotchas.

### Fixed Time Step
`dt = 0.1 s` everywhere. Change it only in `main.m`; every class reads
`dt` as a parameter so nothing is hard-coded.

### Lidar Raycasting (Vectorised)
`RobotAgent.senseEnvironment()` casts all `M` rays simultaneously:

* **AABB slab method** — one matrix multiply per obstacle, `O(M·W)`.
* **Analytic ray-circle** — quadratic formula vectorised over `M` rays per entity.
* Gaussian noise added at the end: `σ = 0.02 m` (tunable via `LidarNoise`).

### Collision Detection
`HospitalEnv.detectCollisions()` runs every `dt` step:
1. **Entity-pair** — O(N²) circle–circle overlap (N ≤ ~20 in typical scenarios).
2. **Entity-wall** — O(N·W) circle-AABB using the standard corner-distance formula.
Each collision is printed to the console and counted in `TotalCollisions`.

---

## Overriding the Robot Controller

The only method you need to replace is `calculateVelocity` in `RobotAgent.m`.

```matlab
function [v, omega] = calculateVelocity(obj, target_x, target_y, lidar_data)
    %  obj        – this robot (pose, radius, limits available)
    %  target_x/y – assigned goal in world frame [m]
    %  lidar_data – struct
    %                 .ranges  [NumRays × 1]  range to nearest obstacle [m]
    %                 .angles  [NumRays × 1]  ray angles (local frame) [rad]
    %
    %  Returns
    %    v     – linear  velocity [m/s]  (clamped to MaxLinSpeed by move())
    %    omega – angular velocity [rad/s] (clamped to MaxAngSpeed)

    % --- YOUR GAME-THEORY / MPC / ORCA LOGIC HERE --- %
    v     = 0;
    omega = 0;
end
```

The method is called once per robot per `dt` step **after** the Lidar scan
is completed. The robot has access to:

| `obj` property | Description |
|---|---|
| `obj.X`, `obj.Y`, `obj.Theta` | Current pose |
| `obj.TargetX`, `obj.TargetY` | Assigned goal (NaN if idle) |
| `obj.LidarRanges` | Latest scan ranges |
| `obj.LidarAngles` | Corresponding local-frame angles |
| `obj.Radius` | Physical radius |
| `obj.MaxLinSpeed` | Speed cap |

The robot does **not** have access to `env`, other robots' poses, or nurse
positions — by design. Everything perceived comes through the Lidar.

---

## Configuration (all in `main.m`)

```matlab
% World (fixed for built-in ward geometry in HospitalEnv)
WORLD_W = 60;  WORLD_H = 40;        % [m]

% Timing
dt           = 0.1;                  % [s]
SIM_DURATION = 120;                  % [s]

% Robots
N_ROBOTS      = 4;
LIDAR_RAYS    = 72;
LIDAR_FOV_DEG = 360;
LIDAR_RANGE   = 8.0;                 % [m]

% Nurses
N_NURSES      = 5;
N_CART_NURSES = 2;

% Task dispatcher
TASK_MEAN_INTERVAL = 5.0;            % [s]
TASK_BATCH_PROB    = 0.10;
```

---

## Ward layout and custom geometry

The default floor plan is a **60 m × 40 m** double-loaded ward (patient rooms
north, main corridor, clinical rooms south, lobby east). `buildDefaultObstacles()`
requires `Width == 60` and `Height == 40`. It fills `WallObstacles`, `BedObstacles`,
and `FurnitureObstacles`, then concatenates them into `obj.Obstacles` (each row
`[centre_x, centre_y, half_width, half_height]`).

Semantic delivery sites live in `buildPOIs()`; `spawnTask()` samples from POIs
marked `taskTarget`. To add a new pickup point, extend `buildPOIs()` and (if
needed) add matching obstacles in `buildDefaultObstacles()`. Room labels and
floor colours are defined in `buildRooms()` and drawn in `initFigure()`.

Lidar, collision detection, and nurse wall-bounce all use `obj.Obstacles`.

---

## Live Metrics (displayed on plot title)

```
T=12.3s | Collisions=2 | AvgTaskTime=18.4s | ActiveTasks=3
```

Full metrics are also printed to the command window at simulation end via
`env.printMetrics()`.

---

## Performance Notes

* Rendering is throttled by `RENDER_EVERY` (default 2 steps → 5 Hz render).
  Increase this number to maximise simulation speed at the cost of visual smoothness.
* Lidar is the main CPU cost at high ray counts. 72 rays × 4 robots ≈ 288
  ray-obstacle tests per step; vectorised, this is ~0.5 ms on a modern laptop.
* For large N (>20 robots), consider replacing the O(N²) collision loop with
  a spatial hash or k-d tree.
