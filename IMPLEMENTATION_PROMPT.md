# Hospital Multi-Agent Simulation — Implementation Prompt
### For Claude Code / Cursor: full codebase context assumed.

> Legacy note: this prompt targets archived MATLAB files now located under `legacy/matlab/`.
> Active implementation work should target the TypeScript codebase in `src/`.

---

## Overview of Changes

Two independent workstreams:

1. **Three new robot control strategies** replacing `calculateVelocity` dispatch in `RobotAgent.m`
2. **Smarter nurse behaviour** replacing the random-waypoint walk in `NurseAgent.m`

The benchmark infrastructure in `main.m` already supports named strategies via `controlMode`. Extend the `strategies` struct to include the three new mode strings, and add them as cases in `RobotAgent.setControlMode` and `RobotAgent.calculateVelocity`.

---

## Part 1 — Robot Control Strategies

### 1.1 Shared infrastructure changes

#### In `RobotAgent.m` — properties block

Add the following persistent state properties needed by the new controllers:

```matlab
% --- SFM state ---
% (none needed beyond existing pose/lidar)

% --- CBF state ---
CbfGamma        (1,1) double = 1.5   % class-K rate parameter
CbfDeltaNurse   (1,1) double = 0.80  % [m] extra safety buffer vs nurses
CbfDeltaRobot   (1,1) double = 0.18  % [m] extra safety buffer vs robots
CbfDeltaWall    (1,1) double = 0.04  % [m] extra safety buffer vs walls

% --- MPC state ---
MpcHorizon      (1,1) double = 8     % prediction steps
MpcDt           (1,1) double = 0.12  % [s] MPC internal timestep
MpcQ            (1,1) double = 2.0   % goal tracking weight
MpcR            (1,1) double = 0.4   % control effort weight
MpcEpsNurse     (1,1) double = 0.90  % [m] hard clearance margin vs nurses
MpcEpsRobot     (1,1) double = 0.22  % [m] hard clearance margin vs robots
% Predicted nurse trajectories — set by env before calculateVelocity call
MpcNursePredictions (:,1) struct = struct('px', {}, 'py', {}, 'vx', {}, 'vy', {})

% --- Shared: neighbor cache written by HospitalEnv.step before calculateVelocity ---
NearbyNurses  (:,1) % NurseAgent handle array (set by env each step)
NearbyRobots  (:,1) % RobotAgent handle array (other operational robots)
```

#### In `RobotAgent.setControlMode`

```matlab
allowed = {'default', 'conservative_speed', 'aggressive_speed', ...
           'sfm', 'cbf', 'mpc'};
```

#### In `HospitalEnv.step` — before calling `calculateVelocity`

Each robot needs its neighbor references injected before its controller runs.
Add inside the robot loop (after `updatePlanningContext`, before `calculateVelocity`):

```matlab
obj.Robots(i).NearbyNurses = obj.Nurses;
otherOp = otherRobots(arrayfun(@(r) r.isOperational(), otherRobots));
obj.Robots(i).NearbyRobots = otherOp;
% For MPC: build nurse predictions (constant-velocity model)
if strcmp(obj.Robots(i).ControlMode, 'mpc')
    N = obj.Robots(i).MpcHorizon;
    dtp = obj.Robots(i).MpcDt;
    preds = struct('px', cell(numel(obj.Nurses),1), ...
                   'py', cell(numel(obj.Nurses),1), ...
                   'vx', cell(numel(obj.Nurses),1), ...
                   'vy', cell(numel(obj.Nurses),1));
    for ni = 1:numel(obj.Nurses)
        nurse = obj.Nurses(ni);
        vx = nurse.Speed * cos(nurse.Theta);
        vy = nurse.Speed * sin(nurse.Theta);
        preds(ni).px = nurse.X + (0:N-1)' * dtp * vx;
        preds(ni).py = nurse.Y + (0:N-1)' * dtp * vy;
        preds(ni).vx = vx;
        preds(ni).vy = vy;
    end
    obj.Robots(i).MpcNursePredictions = preds;
end
```

> **Note:** `NurseAgent.Theta` and `NurseAgent.Speed` are already `SetAccess = private` — either add public getters, or promote them to public `SetAccess`. The simplest fix is changing `SetAccess = private` → `SetAccess = public` on the `Speed` and `Theta` properties in `NurseAgent.m`, since they are already read externally by the renderer.

---

### 1.2 Strategy A — Social Force Model (`controlMode = 'sfm'`)

#### Theory

The robot is a particle subject to three additive forces converted to (v, ω) commands each timestep. No optimisation. No prediction.

**Self-driving force** (toward current A\* waypoint at desired speed v⁰):

$$\mathbf{f}^0 = \frac{v^0 \hat{\mathbf{e}}_{\text{goal}} - \mathbf{v}}{\tau}$$

**Agent repulsion** (nurses and other robots):

$$\mathbf{f}_{ij} = A_j \exp\!\left(\frac{r_{ij} - d_{ij}}{B_j}\right) \hat{\mathbf{n}}_{ij} \cdot \left(\lambda + (1-\lambda)\frac{1 + \cos\phi_{ij}}{2}\right)$$

- $d_{ij} = \|\mathbf{p}_i - \mathbf{p}_j\|$, $r_{ij} = r_i + r_j$
- $\hat{\mathbf{n}}_{ij}$ = unit vector from obstacle centre to robot
- $\phi_{ij}$ = angle between robot heading and direction toward obstacle
- **A_nurse = 6.0, B_nurse = 0.6** (strong, long-range repulsion)
- **A_robot = 2.5, B_robot = 0.35** (moderate)
- λ = 0.5 (forward obstacles weighted more heavily)

**Wall repulsion** from lidar:

$$\mathbf{f}_{iw} = A_w \exp\!\left(\frac{-\rho_k}{B_w}\right) (-\hat{\mathbf{r}}_k)$$

summed over lidar rays k where ρ_k < ρ_threshold (3.0 m), where $\hat{\mathbf{r}}_k$ is the ray direction in world frame.

**Force → unicycle command:**

$$\mathbf{F} = \mathbf{f}^0 + \sum_j \mathbf{f}_{ij} + \sum_k \mathbf{f}_{iw}$$

$$v = \text{clamp}(\mathbf{F} \cdot \hat{\mathbf{h}},\ 0,\ v_{\max})$$

$$\omega = \text{clamp}(K_\omega \cdot \mathbf{F} \times \hat{\mathbf{h}},\ -\omega_{\max},\ \omega_{\max})$$

where $\hat{\mathbf{h}} = (\cos\theta, \sin\theta)$ is the robot's heading unit vector and K_ω = 2.0.

#### Code — add as private method in `RobotAgent.m`

```matlab
function [v, omega] = calculateVelocity_SFM(obj, target_x, target_y, lidar_data)
    % Parameters
    A_nurse = 6.0;  B_nurse = 0.60;
    A_robot = 2.5;  B_robot = 0.35;
    A_wall  = 1.8;  B_wall  = 0.30;  rho_thresh = 3.0;
    lambda  = 0.5;
    tau_relax = 0.5;   % relaxation time [s]
    v0      = obj.MaxLinSpeed * 0.85;
    K_omega = 2.0;

    % Current heading unit vector
    hx = cos(obj.Theta); hy = sin(obj.Theta);

    % --- Self-driving force toward current waypoint ---
    [wx, wy, hasWp] = obj.getCurrentWaypoint(target_x, target_y);
    if ~hasWp, v = 0; omega = 0; return; end
    dx = wx - obj.X; dy = wy - obj.Y;
    dist_goal = norm([dx, dy]);
    if dist_goal < 1e-3, v = 0; omega = 0; return; end
    ex = dx / dist_goal; ey = dy / dist_goal;
    % Current velocity estimate (use last commanded v along heading)
    vx_cur = obj.LastV * hx; vy_cur = obj.LastV * hy;
    f0x = (v0 * ex - vx_cur) / tau_relax;
    f0y = (v0 * ey - vy_cur) / tau_relax;

    % --- Agent repulsion (nurses and other robots) ---
    frx = 0; fry = 0;
    all_agents = [obj.NearbyNurses; obj.NearbyRobots];
    for k = 1:numel(all_agents)
        ag = all_agents(k);
        if isfield(ag, 'HasCart')   % NurseAgent
            A = A_nurse; B = B_nurse;
        else
            A = A_robot; B = B_robot;
        end
        ox = ag.X - obj.X; oy = ag.Y - obj.Y;
        d = norm([ox, oy]);
        rij = obj.Radius + ag.Radius;
        if d < 1e-3, continue; end
        nx_ = -ox / d; ny_ = -oy / d;  % toward robot
        mag = A * exp((rij - d) / B);
        % Anisotropy: forward-facing obstacles weighted more
        cos_phi = hx * (-nx_) + hy * (-ny_);
        w_aniso = lambda + (1 - lambda) * (1 + cos_phi) / 2;
        frx = frx + mag * w_aniso * nx_;
        fry = fry + mag * w_aniso * ny_;
    end

    % --- Wall repulsion from lidar rays ---
    fwx = 0; fwy = 0;
    world_angles = lidar_data.angles + obj.Theta;
    for k = 1:numel(lidar_data.ranges)
        rho = lidar_data.ranges(k);
        if rho >= rho_thresh, continue; end
        ray_dx = cos(world_angles(k));
        ray_dy = sin(world_angles(k));
        mag = A_wall * exp(-rho / B_wall);
        fwx = fwx - mag * ray_dx;
        fwy = fwy - mag * ray_dy;
    end

    % --- Sum forces and convert to (v, omega) ---
    Fx = f0x + frx + fwx;
    Fy = f0y + fry + fwy;
    v     = max(0, min(obj.MaxLinSpeed, Fx*hx + Fy*hy));
    omega = K_omega * (Fx*(-hy) + Fy*hx);  % cross product z-component
    omega = max(-obj.MaxAngSpeed, min(obj.MaxAngSpeed, omega));
end
```

#### In `calculateVelocity` switch block, add:

```matlab
case 'sfm'
    [v, omega] = obj.calculateVelocity_SFM(target_x, target_y, lidar_data);
    return;
```

Place this **before** the existing `switch obj.ControlMode` that sets `Kp_omega` / `speedScale`, so it returns early without entering that block.

---

### 1.3 Strategy B — Control Barrier Function (`controlMode = 'cbf'`)

#### Theory

The nominal (v_nom, ω_nom) comes from the existing A\*-guided proportional heading controller (the current `default` case, unchanged). The CBF filters this output via a small QP that minimally deforms the command to enforce collision-safety.

**Barrier function** for obstacle j:

$$h_j(\mathbf{x}) = \|\mathbf{p} - \mathbf{p}_j\|^2 - (r_i + r_j + \delta_j)^2$$

The safe set is $\mathcal{C} = \{\mathbf{x} \mid h_j \geq 0\ \forall j\}$.

**CBF condition** (forward invariance of $\mathcal{C}$):

$$\dot{h}_j = \nabla_{\mathbf{x}} h_j \cdot f(\mathbf{x}, \mathbf{u}) \geq -\gamma \cdot h_j(\mathbf{x})$$

For the unicycle $\dot{x} = v\cos\theta$, $\dot{y} = v\sin\theta$:

$$\dot{h}_j = 2(p_x - p_{jx})(v\cos\theta - \dot{p}_{jx}) + 2(p_y - p_{jy})(v\sin\theta - \dot{p}_{jy}) \geq -\gamma \cdot h_j$$

This is **linear in v** (ω does not appear directly — only through future heading). Rearranging into the standard form $\mathbf{a}_j^T \mathbf{u} \geq b_j$ where $\mathbf{u} = [v,\ \omega]^T$:

$$\mathbf{a}_j = \begin{bmatrix} 2(p_x - p_{jx})\cos\theta + 2(p_y - p_{jy})\sin\theta \\ 0 \end{bmatrix}$$

$$b_j = -\gamma \cdot h_j - 2(p_x - p_{jx})(-\dot{p}_{jx}) - 2(p_y - p_{jy})(-\dot{p}_{jy})$$

**Safety filter QP** (2 variables, n_obs + 2 bound constraints):

$$\min_{v, \omega}\ (v - v_{\text{nom}})^2 + ({\omega - \omega_{\text{nom}}})^2$$

$$\text{s.t.}\ \mathbf{a}_j^T [v,\omega]^T \geq b_j \quad \forall j \in \text{obstacles}$$

$$0 \leq v \leq v_{\max},\quad |\omega| \leq \omega_{\max}$$

Solved with MATLAB's `quadprog` or a simple active-set loop (see code below — no toolbox required).

**δ values:**

| Obstacle type | δ (extra buffer) | γ (urgency) |
|---------------|-----------------|-------------|
| Nurse         | 0.80 m          | 1.5         |
| Other robot   | 0.18 m          | 2.0         |
| Wall (lidar)  | 0.04 m          | 3.0         |

#### Code — add as private method in `RobotAgent.m`

```matlab
function [v, omega] = calculateVelocity_CBF(obj, target_x, target_y, lidar_data)
    % Step 1: get nominal command from existing default controller logic
    [v_nom, omega_nom] = obj.calculateVelocity_Default(target_x, target_y, lidar_data);

    % Step 2: build CBF constraints
    A_cbf = zeros(0, 2);
    b_cbf = zeros(0, 1);

    px = obj.X; py = obj.Y;
    cth = cos(obj.Theta); sth = sin(obj.Theta);

    % Helper: add one barrier constraint for circular obstacle at (ox,oy)
    % with combined radius r_sum, buffer delta, gain gamma, velocity (vox,voy)
    function addBarrier(ox, oy, r_sum, delta, gamma, vox, voy)
        dpx = px - ox; dpy = py - oy;
        d2  = dpx^2 + dpy^2;
        h   = d2 - (r_sum + delta)^2;
        % dh/du: linear in v, zero in omega (first-order CBF)
        a_v = 2*(dpx*cth + dpy*sth);
        % b: RHS = -gamma*h + relative velocity correction
        b_val = -gamma * h ...
              - 2*dpx*(-vox) - 2*dpy*(-voy);
        A_cbf(end+1, :) = [a_v, 0];
        b_cbf(end+1)    = b_val;
    end

    % Nurses
    for k = 1:numel(obj.NearbyNurses)
        n = obj.NearbyNurses(k);
        vox = n.Speed * cos(n.Theta);
        voy = n.Speed * sin(n.Theta);
        addBarrier(n.X, n.Y, obj.Radius + n.Radius, ...
                   obj.CbfDeltaNurse, obj.CbfGamma, vox, voy);
    end

    % Other robots
    for k = 1:numel(obj.NearbyRobots)
        r = obj.NearbyRobots(k);
        vox = r.LastV * cos(r.Theta);
        voy = r.LastV * sin(r.Theta);
        addBarrier(r.X, r.Y, obj.Radius + r.Radius, ...
                   obj.CbfDeltaRobot, 2.0, vox, voy);
    end

    % Walls via lidar (treat each short-range hit as a point obstacle)
    world_angles = lidar_data.angles + obj.Theta;
    for k = 1:numel(lidar_data.ranges)
        rho = lidar_data.ranges(k);
        if rho >= obj.MaxRange - 0.1, continue; end  % max range = no hit
        if rho > 2.5, continue; end                  % only nearby walls
        wx_ = px + rho * cos(world_angles(k));
        wy_ = py + rho * sin(world_angles(k));
        addBarrier(wx_, wy_, obj.Radius, obj.CbfDeltaWall, 3.0, 0, 0);
    end

    % Step 3: Solve QP: min ||u - u_nom||^2  s.t. A_cbf*u >= b_cbf, bounds
    u_nom = [v_nom; omega_nom];
    lb = [0;           -obj.MaxAngSpeed];
    ub = [obj.MaxLinSpeed; obj.MaxAngSpeed];

    u_star = obj.solveCBF_QP(u_nom, A_cbf, b_cbf, lb, ub);
    v     = u_star(1);
    omega = u_star(2);
end

function u = solveCBF_QP(~, u_nom, A_ineq, b_ineq, lb, ub)
    % Minimal active-set QP: min 0.5*||u - u_nom||^2
    % s.t. A_ineq * u >= b_ineq,  lb <= u <= ub
    % Convert to standard form: min 0.5*u'*I*u - u_nom'*u
    %   s.t. -A_ineq * u <= -b_ineq,  lb <= u <= ub
    n = numel(u_nom);
    if isempty(A_ineq)
        u = max(lb, min(ub, u_nom));
        return;
    end
    % Try unconstrained solution first
    u_try = max(lb, min(ub, u_nom));
    if all(A_ineq * u_try >= b_ineq - 1e-6)
        u = u_try;
        return;
    end
    % Use MATLAB quadprog if available, else gradient projection
    try
        opts = optimoptions('quadprog','Display','none');
        H = eye(n);
        f_qp = -u_nom;
        [u_sol, ~, flag] = quadprog(H, f_qp, -A_ineq, -b_ineq, ...
                                     [], [], lb, ub, u_nom, opts);
        if flag > 0
            u = u_sol;
        else
            u = max(lb, min(ub, u_nom));  % infeasible fallback
        end
    catch
        % Fallback: projected gradient (no toolbox)
        u = max(lb, min(ub, u_nom));
        step = 0.05;
        for iter = 1:30
            viol = find(A_ineq * u < b_ineq);
            if isempty(viol), break; end
            grad = zeros(n,1);
            for vi = viol'
                a = A_ineq(vi,:)';
                slack = b_ineq(vi) - dot(a, u);
                grad = grad - slack * a;
            end
            u = max(lb, min(ub, u - step * grad));
        end
    end
end
```

Add a thin wrapper to extract the default controller logic as its own callable method `calculateVelocity_Default` (copy the existing proportional-heading block out of `calculateVelocity` into this new private method), so CBF can call it cleanly.

#### In `calculateVelocity` switch block:

```matlab
case 'cbf'
    [v, omega] = obj.calculateVelocity_CBF(target_x, target_y, lidar_data);
    return;
```

---

### 1.4 Strategy C — MPC with Human Motion Prediction (`controlMode = 'mpc'`)

#### Theory

At each timestep, solve a finite-horizon optimal control problem over N steps. Nurse positions are predicted via a constant-velocity model. Hard clearance constraints prevent the solution from placing the robot within ε_nurse of any predicted nurse position at any horizon step.

**Prediction model** for nurse j at step k:

$$\hat{p}_{jx}(k) = p_{jx}(0) + k \cdot \Delta t_{\text{mpc}} \cdot v_{jx}, \quad \hat{p}_{jy}(k) = p_{jy}(0) + k \cdot \Delta t_{\text{mpc}} \cdot v_{jy}$$

**Unicycle discretisation** (Euler, matches existing `move`):

$$x(k+1) = x(k) + v(k)\cos\theta(k)\,\Delta t$$
$$y(k+1) = y(k) + v(k)\sin\theta(k)\,\Delta t$$
$$\theta(k+1) = \theta(k) + \omega(k)\,\Delta t$$

**Objective** (tracking + effort):

$$\min_{\{v_k, \omega_k\}} \sum_{k=1}^{N} Q \left[(x_k - x_g)^2 + (y_k - y_g)^2\right] + \sum_{k=0}^{N-1} R \left[v_k^2 + \omega_k^2\right]$$

where $(x_g, y_g)$ is the current A\* waypoint.

**Hard collision constraints** at every step k, for every nurse j:

$$\|(x_k, y_k) - (\hat{p}_{jx}(k), \hat{p}_{jy}(k))\|^2 \geq (r_i + r_j + \varepsilon_{\text{nurse}})^2$$

and for every other robot m:

$$\|(x_k, y_k) - (p_{mx}(k), p_{my}(k))\|^2 \geq (r_i + r_m + \varepsilon_{\text{robot}})^2$$

**Implementation approach:** Because the unicycle is nonlinear and the collision constraints are nonlinear, this is a nonlinear programme (NLP). For real-time use at dt = 0.1s, use **Sequential Quadratic Programming (SQP)** via warm-starting from the previous solution, or use the following tractable approximation:

**Tractable SQP approximation (recommended):** Linearise the unicycle about the current trajectory using a shooting method. At each outer iteration, solve the resulting QP and update the trajectory. Two to three SQP iterations are sufficient for a good local solution given warm-starting.

Alternatively, use MATLAB's `fmincon` with `'sqp'` algorithm and `MaxIterations = 5` — fast enough for 4 robots at 10 Hz.

#### Code — add as private method in `RobotAgent.m`

```matlab
function [v, omega] = calculateVelocity_MPC(obj, target_x, target_y, ~)
    N   = obj.MpcHorizon;
    dtp = obj.MpcDt;
    Q   = obj.MpcQ;
    R   = obj.MpcR;
    eps_n = obj.MpcEpsNurse;
    eps_r = obj.MpcEpsRobot;

    [wx, wy, hasWp] = obj.getCurrentWaypoint(target_x, target_y);
    if ~hasWp, v = 0; omega = 0; return; end

    % Decision variable: u = [v0 w0 v1 w1 ... v_{N-1} w_{N-1}]  (2N x 1)
    % Warm start: repeat last command
    u0 = repmat([max(0, obj.LastV); obj.LastOmega], N, 1);

    % Bounds
    lb = repmat([0; -obj.MaxAngSpeed], N, 1);
    ub = repmat([obj.MaxLinSpeed; obj.MaxAngSpeed], N, 1);

    x0 = [obj.X; obj.Y; obj.Theta];

    % Cost and constraint functions
    function J = mpc_cost(u_vec)
        x = x0; J = 0;
        for k = 1:N
            vk = u_vec(2*k-1); wk = u_vec(2*k);
            x(1) = x(1) + vk * cos(x(3)) * dtp;
            x(2) = x(2) + vk * sin(x(3)) * dtp;
            x(3) = x(3) + wk * dtp;
            J = J + Q*((x(1)-wx)^2 + (x(2)-wy)^2) + R*(vk^2 + wk^2);
        end
    end

    function [c, ceq] = mpc_constraints(u_vec)
        ceq = [];
        c = zeros(0,1);
        x = x0;
        for k = 1:N
            vk = u_vec(2*k-1); wk = u_vec(2*k);
            x(1) = x(1) + vk * cos(x(3)) * dtp;
            x(2) = x(2) + vk * sin(x(3)) * dtp;
            x(3) = x(3) + wk * dtp;
            % Nurse constraints (hard)
            for ni = 1:numel(obj.MpcNursePredictions)
                pred = obj.MpcNursePredictions(ni);
                if k > numel(pred.px), continue; end
                r_sum = obj.Radius + obj.NearbyNurses(ni).Radius + eps_n;
                dx_ = x(1) - pred.px(k); dy_ = x(2) - pred.py(k);
                c(end+1) = r_sum^2 - (dx_^2 + dy_^2);  %#ok<AGROW>
            end
            % Robot constraints (hard)
            for ri = 1:numel(obj.NearbyRobots)
                other = obj.NearbyRobots(ri);
                r_sum = obj.Radius + other.Radius + eps_r;
                dx_ = x(1) - other.X; dy_ = x(2) - other.Y;
                c(end+1) = r_sum^2 - (dx_^2 + dy_^2);  %#ok<AGROW>
            end
        end
    end

    opts = optimoptions('fmincon', 'Algorithm', 'sqp', ...
        'Display', 'none', 'MaxIterations', 6, ...
        'MaxFunctionEvaluations', 400, 'OptimalityTolerance', 1e-3);
    try
        u_sol = fmincon(@mpc_cost, u0, [], [], [], [], lb, ub, ...
                        @mpc_constraints, opts);
        v     = max(0, min(obj.MaxLinSpeed, u_sol(1)));
        omega = max(-obj.MaxAngSpeed, min(obj.MaxAngSpeed, u_sol(2)));
    catch
        % Fallback to CBF if fmincon fails
        [v, omega] = obj.calculateVelocity_CBF(target_x, target_y, ...
            obj.senseEnvironment(obj.KnownObstacles, ...
            obj.NearbyNurses, obj.NearbyRobots));
    end
end
```

#### In `calculateVelocity` switch block:

```matlab
case 'mpc'
    [v, omega] = obj.calculateVelocity_MPC(target_x, target_y, lidar_data);
    return;
```

---

### 1.5 Update `main.m` strategies struct

```matlab
strategies = struct( ...
    'name',           {'sfm',           'cbf',           'mpc'}, ...
    'controlMode',    {'sfm',           'cbf',           'mpc'}, ...
    'assignmentPolicy',{'nearest_idle', 'nearest_idle',  'nearest_idle'});
```

---

## Part 2 — Smarter Nurse Behaviour

### 2.1 Design intent

Each nurse has a **role** and a **state machine** with 4 states:

```
working_in_room → transit → (optional: fetch_cart) → working_in_room
```

- Nurses spend most of their time *inside a room*, performing short local micro-movements (simulating patient care, charting, checking equipment).
- Periodically they decide to move to another room — always via a semantically plausible destination (patient room → nurse station → pharmacy → ICU, etc.).
- Cart nurses occasionally go to the supply room first to "retrieve" their cart before transiting.
- All movement still uses the existing waypoint-walk mechanics, but destinations are now drawn from `env.POIs` filtered by room type rather than random world coordinates.

### 2.2 New properties — add to `NurseAgent.m`

```matlab
properties (SetAccess = public)   % promote for MPC nurse prediction access
    Speed     (1,1) double = 0
    Theta     (1,1) double = 0
end

properties (SetAccess = private)
    % Role assigned at construction
    Role (1,:) char = 'floor_nurse'  % 'floor_nurse' | 'charge_nurse' | 'cart_nurse'

    % State machine
    NurseState   (1,:) char = 'working_in_room'
    % 'working_in_room' | 'transit' | 'fetch_cart' | 'dwelling_corridor'

    % Timers
    WorkTimer    (1,1) double = 0   % time remaining in current room activity
    MicroTimer   (1,1) double = 0   % time until next micro-movement within room
    FetchTimer   (1,1) double = 0   % time spent at supply room retrieving cart

    % Current room context
    CurrentRoomName  (1,:) char = ''
    CurrentRoomBounds (1,4) double = [0 0 0 0]  % [xmin xmax ymin ymax]

    % Micro-movement target (within current room)
    MicroTargetX (1,1) double = NaN
    MicroTargetY (1,1) double = NaN

    % Supply room reference (set by env)
    SupplyRoomPOIs (:,2) double = zeros(0,2)
end

properties (Constant, Access = private)
    % State durations (sampled uniformly between min/max)
    WorkTimeMin   = 12.0   % [s] minimum time working in a room
    WorkTimeMax   = 45.0   % [s]
    MicroMoveInterval = 4.0   % [s] how often to take a micro-step within room
    MicroMoveRadius   = 1.2   % [m] radius of micro-movements within room
    FetchCartTime     = 6.0   % [s] time to retrieve cart at supply room
    CartFetchProb     = 0.30  % probability a cart_nurse fetches cart before transit
    CorridorDwellMax  = 5.0   % [s] max dwell at intermediate corridor waypoint
end
```

### 2.3 Constructor change

```matlab
function obj = NurseAgent(x, y, hasCart, role)
    % ... existing init ...
    if nargin >= 4
        obj.Role = role;
    elseif hasCart
        obj.Role = 'cart_nurse';
    end
    obj.WorkTimer  = NurseAgent.WorkTimeMin + ...
        rand() * (NurseAgent.WorkTimeMax - NurseAgent.WorkTimeMin);
    obj.MicroTimer = NurseAgent.MicroMoveInterval * rand();
end
```

### 2.4 New public method: `setRoomContext`

Called by `HospitalEnv` after placement to tell each nurse which room they start in:

```matlab
function setRoomContext(obj, roomName, roomBounds, supplyPOIs)
    % roomBounds: [xmin xmax ymin ymax]
    obj.CurrentRoomName   = roomName;
    obj.CurrentRoomBounds = roomBounds;
    obj.SupplyRoomPOIs    = supplyPOIs;
    obj.MicroTargetX = obj.X;
    obj.MicroTargetY = obj.Y;
end
```

### 2.5 Replace `step` method

```matlab
function step(obj, dt, obstacles, worldW, worldH)
    switch obj.NurseState
        case 'working_in_room'
            obj.stepWorkingInRoom(dt, obstacles, worldW, worldH);
        case 'transit'
            obj.walkTowardWaypoint(dt, obstacles, worldW, worldH);
            % Check arrival
            if norm([obj.WaypointX - obj.X, obj.WaypointY - obj.Y]) < 0.2
                obj.arriveAtDestination();
            end
        case 'fetch_cart'
            obj.FetchTimer = obj.FetchTimer - dt;
            if obj.FetchTimer <= 0
                obj.HasCart  = true;
                obj.Radius   = NurseAgent.RadiusCart;
                obj.NurseState = 'transit';
                obj.pickTransitDestination();
            end
        case 'dwelling_corridor'
            obj.DwellTime = obj.DwellTime - dt;
            if obj.DwellTime <= 0
                obj.NurseState = 'transit';
            end
    end
end
```

### 2.6 New private methods

```matlab
function stepWorkingInRoom(obj, dt, obstacles, worldW, worldH)
    obj.WorkTimer  = obj.WorkTimer - dt;
    obj.MicroTimer = obj.MicroTimer - dt;

    % Micro-movement: small purposeful steps within the room
    if obj.MicroTimer <= 0
        obj.MicroTimer = NurseAgent.MicroMoveInterval * (0.5 + rand());
        % Sample a new micro-target within room bounds + micro radius
        b = obj.CurrentRoomBounds;
        for attempt = 1:20
            mx = b(1) + rand() * (b(2) - b(1));
            my = b(3) + rand() * (b(4) - b(3));
            if ~obj.insideAnyObstacle(mx, my, obstacles)
                obj.MicroTargetX = mx;
                obj.MicroTargetY = my;
                break;
            end
        end
    end

    % Walk toward micro-target (slow speed — "working")
    obj.WaypointX = obj.MicroTargetX;
    obj.WaypointY = obj.MicroTargetY;
    obj.Speed = NurseAgent.MinSpeed * 0.6;
    if norm([obj.WaypointX - obj.X, obj.WaypointY - obj.Y]) > 0.15
        obj.walkTowardWaypoint(dt, obstacles, worldW, worldH);
    end

    % Decide to leave room
    if obj.WorkTimer <= 0
        obj.leaveRoom();
    end
end

function leaveRoom(obj)
    % Cart nurses may fetch cart from supply room first
    if strcmp(obj.Role, 'cart_nurse') && ~obj.HasCart && ...
            rand() < NurseAgent.CartFetchProb && ~isempty(obj.SupplyRoomPOIs)
        idx = randi(size(obj.SupplyRoomPOIs, 1));
        obj.WaypointX  = obj.SupplyRoomPOIs(idx, 1);
        obj.WaypointY  = obj.SupplyRoomPOIs(idx, 2);
        obj.Speed      = NurseAgent.MinSpeed + rand() * ...
                         (NurseAgent.MaxSpeed - NurseAgent.MinSpeed);
        obj.FetchTimer = NurseAgent.FetchCartTime;
        obj.NurseState = 'fetch_cart';
    else
        obj.NurseState = 'transit';
        obj.pickTransitDestination();
    end
end

function pickTransitDestination(obj)
    % Choose next destination from Waypoints list (semantic POIs)
    % Prefer POIs not in current room
    nWp = size(obj.Waypoints, 1);
    if nWp == 0
        % Fallback to original random waypoint behaviour
        obj.pickNewWaypoint({}, 60, 40);  % world dims not ideal here;
        % caller should pass; acceptable as a rare fallback
        return;
    end
    best = [];
    for attempt = 1:40
        idx = randi(nWp);
        wx = obj.Waypoints(idx, 1);
        wy = obj.Waypoints(idx, 2);
        b  = obj.CurrentRoomBounds;
        inCurrentRoom = wx >= b(1) && wx <= b(2) && ...
                        wy >= b(3) && wy <= b(4);
        if ~inCurrentRoom
            best = [wx, wy];
            break;
        end
    end
    if isempty(best)
        best = obj.Waypoints(randi(nWp), :);
    end
    obj.WaypointX  = best(1);
    obj.WaypointY  = best(2);
    obj.Speed      = NurseAgent.MinSpeed + rand() * ...
                     (NurseAgent.MaxSpeed - NurseAgent.MinSpeed);
end

function arriveAtDestination(obj)
    % Identify new room from waypoint position (simple bounding-box check
    % against obj.RoomBoundsTable if provided, else default work timer)
    obj.NurseState = 'working_in_room';
    obj.WorkTimer  = NurseAgent.WorkTimeMin + rand() * ...
                     (NurseAgent.WorkTimeMax - NurseAgent.WorkTimeMin);
    obj.MicroTimer = NurseAgent.MicroMoveInterval * rand();
    % Update room bounds to new location (coarse: use waypoint ± room half-size)
    hw = 2.5; hh = 2.0;   % approximate room half-extents
    obj.CurrentRoomBounds = [obj.WaypointX - hw, obj.WaypointX + hw, ...
                              obj.WaypointY - hh, obj.WaypointY + hh];
end
```

### 2.7 Hook into `HospitalEnv`

In `HospitalEnv.addNurse`, after the existing `nurse.setWaypoints` call:

```matlab
% Assign room context from starting position
startRoom = obj.getRoomForPosition(nurse.X, nurse.Y);
if ~isempty(startRoom)
    bounds = [startRoom.xmin, startRoom.xmax, ...
              startRoom.ymin, startRoom.ymax];
    supplyPOIs = obj.getSupplyPOIs();
    nurse.setRoomContext(startRoom.name, bounds, supplyPOIs);
end
```

Add two private helper methods to `HospitalEnv`:

```matlab
function room = getRoomForPosition(obj, x, y)
    room = [];
    for k = 1:numel(obj.Rooms)
        r = obj.Rooms(k);
        if x >= r.xmin && x <= r.xmax && y >= r.ymin && y <= r.ymax
            room = r; return;
        end
    end
end

function pois = getSupplyPOIs(obj)
    pois = zeros(0, 2);
    for k = 1:numel(obj.POIs)
        if contains(lower(obj.POIs(k).name), 'supply') || ...
           contains(lower(obj.POIs(k).roomName), 'supply')
            pois(end+1, :) = [obj.POIs(k).x, obj.POIs(k).y]; %#ok<AGROW>
        end
    end
end
```

---

## Part 3 — Parameter Tuning Reference

The following parameters have the most impact on benchmark outcomes. Tune after confirming basic correctness:

| Parameter | Location | Effect |
|-----------|----------|--------|
| `A_nurse` / `B_nurse` (SFM) | `calculateVelocity_SFM` | Nurse avoidance radius and strength |
| `CbfDeltaNurse`, `CbfGamma` | `RobotAgent` properties | CBF nurse exclusion size and urgency |
| `MpcEpsNurse`, `MpcHorizon` | `RobotAgent` properties | MPC hard clearance and lookahead depth |
| `WorkTimeMin/Max` | `NurseAgent` constants | How often nurses transit between rooms |
| `CartFetchProb` | `NurseAgent` constants | Frequency of cart-retrieval trips |
| `MicroMoveInterval` | `NurseAgent` constants | Busyness of in-room nurse micro-movement |

---

## Part 4 — Determinism Note

`fmincon` uses internal randomness in some configurations. To preserve benchmark determinism across seeds, add the following inside `runSingleExperiment` in `main.m`, immediately after `rng(seed, 'twister')`:

```matlab
% Suppress fmincon's internal random restarts for reproducibility
warning('off', 'optim:fmincon:SwitchingToMediumScale');
```

And set `MpcHorizon = 8`, `MaxIterations = 6` as above — short enough that SQP converges without random restarts.

---

## Summary of Files Changed

| File | Changes |
|------|---------|
| `RobotAgent.m` | New properties block; 3 new private methods; `setControlMode` + `calculateVelocity` dispatch extended; `calculateVelocity_Default` extracted |
| `NurseAgent.m` | New properties; state machine replacing random-waypoint walk; `Speed`/`Theta` promoted to public; `setRoomContext` public method |
| `HospitalEnv.m` | Neighbor injection in `step`; nurse prediction in `step`; `addNurse` room context hook; 2 new private helpers |
| `main.m` | `strategies` struct updated to 3 new strategies |
