# Hospital Simulation — Scoring System Implementation Prompt
### For Claude Code / Cursor: full codebase context assumed.

> Legacy note: this prompt targets archived MATLAB files now located under `legacy/matlab/`.
> Active implementation work should target the TypeScript codebase in `src/`.

---

## Overview

Add a decomposable scalar cost function `S` to the benchmark pipeline. Changes are
confined to `main.m` only — no class files need modification. The score is computed
inside `runSingleExperiment` and printed inside `printLeaderboard`.

---

## Cost Function (reference)

$$S = \left(C_{\text{task}} + w_{rw} C_{rw} + w_{rr} C_{rr}\right) \cdot (1 + C_{rn})$$

Where:

$$C_{\text{task}} = \frac{1}{N_c + N_u} \sum_{i} \frac{\tilde{\tau}_i}{T}, \qquad
\tilde{\tau}_i = \begin{cases} \tau_i & \text{completed} \\ 1.4\,(T - s_i) & \text{uncompleted} \end{cases}$$

| Constant | Value | Rationale |
|----------|-------|-----------|
| $\alpha$ (frustration multiplier) | 1.4 | Uncompleted tasks 40% worse than a just-completed equivalent |
| $w_{rw}$ (wall collision weight) | 0.042 | Calibrated: 5s extra task time ≈ 1 wall collision |
| $w_{rr}$ (robot–robot weight) | 0.170 | ~4× wall weight; run survives 1–2 but they hurt |
| $\beta$ (human multiplier per nurse collision) | 1.0 | Each nurse collision adds 1.0 to the multiplier (linear) |

Lower score = better. A run with zero nurse collisions has multiplier 1.0 (no effect).
Each nurse collision doubles, triples, etc. the combined task + collision cost.

---

## Step 1 — Extend the `result` struct in `runSingleExperiment`

### 1a. The `env` object already exposes `CompletionTimes` and `TotalCollisions`.
You need two additional pieces of data not currently captured:

**Spawn times of uncompleted tasks.** At the end of the run, iterate `env.Tasks` and
collect `spawnTime` for any task whose `assignedTo` robot has not completed it
(i.e. it is still in the queue or assigned but unfinished). The simplest proxy:
a task is uncompleted if its `id` does not correspond to any logged completion.
Since `env.CompletionTimes` is a plain vector of durations (not IDs), use task
count: tasks completed = `numel(env.CompletionTimes)`, tasks uncompleted =
all remaining entries in `env.Tasks` that were spawned but not completed.

**Nurse collision count split from total.** `env.TotalCollisions` is currently a
single counter. You need robot–nurse and robot–robot (and optionally robot–wall)
tracked separately. See Step 1b.

### 1b. Split collision counters in `HospitalEnv`

In `HospitalEnv.m`, add three new properties alongside `TotalCollisions`:

```matlab
CollisionsRobotNurse  (1,1) double = 0
CollisionsRobotRobot  (1,1) double = 0
CollisionsRobotWall   (1,1) double = 0
```

In `HospitalEnv.detectCollisions` (the private method that increments
`TotalCollisions`), increment the appropriate sub-counter at the same site:

- Robot vs NurseAgent circle overlap → `CollisionsRobotNurse`
- Robot vs RobotAgent circle overlap → `CollisionsRobotRobot`
- Robot vs AABB obstacle (wall/bed/furniture) → `CollisionsRobotWall`

Keep `TotalCollisions = CollisionsRobotNurse + CollisionsRobotRobot + CollisionsRobotWall`
for backward compatibility with any existing log output.

### 1c. Collect uncompleted task spawn times

At the end of `runSingleExperiment`, after the simulation loop and before building
`result`, add:

```matlab
% Collect spawn times of uncompleted tasks.
% A task is uncompleted if it is still present in env.Tasks at end of run.
% Completed tasks are removed from env.Tasks by checkTaskCompletion, so
% every entry remaining is either unassigned or assigned-but-unfinished.
uncompletedSpawnTimes = zeros(numel(env.Tasks), 1);
for ti = 1:numel(env.Tasks)
    uncompletedSpawnTimes(ti) = env.Tasks(ti).spawnTime;
end
```

> **Verify assumption:** confirm that `HospitalEnv.checkTaskCompletion` removes
> completed tasks from `env.Tasks` (sets them to done and removes, or marks them).
> If it does not remove them, filter by completion status instead. Adjust this
> collection logic to match actual behaviour.

### 1d. Extend the `result` struct fields

```matlab
result.strategy              = strategyCfg.name;
result.seed                  = seed;
result.collisions            = env.TotalCollisions;
result.collisionsNurse       = env.CollisionsRobotNurse;
result.collisionsRobot       = env.CollisionsRobotRobot;
result.collisionsWall        = env.CollisionsRobotWall;
result.avgTime               = avgTime;           % existing — keep
result.tasksCompleted        = numel(env.CompletionTimes);
result.completionTimes       = env.CompletionTimes;       % full vector
result.uncompletedSpawnTimes = uncompletedSpawnTimes;     % new
result.simDuration           = simCfg.SIM_DURATION;
result.wallTime              = wallTime;
```

Also extend the `repmat(struct(...))` initialiser near the top of `main.m` that
pre-allocates `allResults` to include the new fields with sensible defaults:

```matlab
allResults = repmat(struct( ...
    'strategy',              '', ...
    'seed',                  0, ...
    'collisions',            0, ...
    'collisionsNurse',       0, ...
    'collisionsRobot',       0, ...
    'collisionsWall',        0, ...
    'avgTime',               Inf, ...
    'tasksCompleted',        0, ...
    'completionTimes',       {zeros(0,1)}, ...
    'uncompletedSpawnTimes', {zeros(0,1)}, ...
    'simDuration',           0, ...
    'wallTime',              0), ...
    numel(strategies) * numel(seedList), 1);
```

---

## Step 2 — Add `computeScore` helper function in `main.m`

Add this as a new local function at the bottom of `main.m`, alongside
`runSingleExperiment` and `printLeaderboard`:

```matlab
function S = computeScore(result)
    % computeScore  Decomposable scalar cost for one (strategy, seed) run.
    %
    %   S = (C_task + w_rw*C_rw + w_rr*C_rr) * (1 + C_rn)
    %
    %   Lower is better.

    % --- Constants ---
    ALPHA   = 1.4;    % frustration multiplier for uncompleted tasks
    W_RW    = 0.042;  % robot-wall collision weight
    W_RR    = 0.170;  % robot-robot collision weight
    BETA    = 1.0;    % per-nurse-collision multiplier increment

    T = result.simDuration;

    % --- Effective service times (normalised by T) ---
    tau_completed   = result.completionTimes / T;           % [Nc x 1]
    tau_uncompleted = ALPHA * (T - result.uncompletedSpawnTimes) / T; % [Nu x 1]

    all_tau = [tau_completed; tau_uncompleted];
    N_total = numel(all_tau);

    if N_total == 0
        % Edge case: no tasks ever spawned (pathological seed)
        C_task = 1.0;   % maximum normalised cost
    else
        C_task = mean(all_tau);
    end

    % --- Collision penalties (additive, dimensionless) ---
    P_collision = W_RW * result.collisionsWall + ...
                  W_RR * result.collisionsRobot;

    % --- Human safety multiplier ---
    M_human = 1 + BETA * result.collisionsNurse;

    % --- Final score ---
    S = (C_task + P_collision) * M_human;
end
```

---

## Step 3 — Attach score to each result in the experiment loop

Inside the `for seed = seedList` loop in `main.m`, after the existing
`allResults(resultIdx) = runResult;` line, add:

```matlab
allResults(resultIdx).score = computeScore(allResults(resultIdx));

fprintf(['[SCORE] strategy=%-24s seed=%3d  ' ...
         'S=%.4f  (C_task=%.3f  P_wall=%.3f  P_robot=%.3f  M_human=%.1fx)\n'], ...
    allResults(resultIdx).strategy, ...
    allResults(resultIdx).seed, ...
    allResults(resultIdx).score, ...
    mean([allResults(resultIdx).completionTimes; ...
          1.4*(allResults(resultIdx).simDuration - ...
               allResults(resultIdx).uncompletedSpawnTimes)]) ...
          / allResults(resultIdx).simDuration, ...
    0.042 * allResults(resultIdx).collisionsWall, ...
    0.170 * allResults(resultIdx).collisionsRobot, ...
    1 + allResults(resultIdx).collisionsNurse);
```

Also add `'score', 0` to the `allResults` pre-allocation struct in the constants
block.

---

## Step 4 — Rewrite `printLeaderboard` to use the score

Replace the existing `printLeaderboard` function with the following:

```matlab
function printLeaderboard(allResults, strategies)

    % --- Per-seed detail ---
    fprintf('\n===== PER-SEED RESULTS =====\n');
    fprintf('  %-24s %5s %6s %5s %5s %5s %8s %8s\n', ...
        'strategy', 'seed', 'S', 'C_rn', 'C_rr', 'C_rw', ...
        'avgTime', 'done');
    for i = 1:numel(allResults)
        rr = allResults(i);
        fprintf('  %-24s %5d %6.3f %5d %5d %5d %8.2f %8d\n', ...
            rr.strategy, rr.seed, rr.score, ...
            rr.collisionsNurse, rr.collisionsRobot, rr.collisionsWall, ...
            rr.avgTime, rr.tasksCompleted);
    end

    % --- Per-strategy summary (mean across seeds) ---
    summary = repmat(struct( ...
        'strategy',        '', ...
        'meanScore',       Inf, ...
        'meanCtask',       Inf, ...
        'meanPwall',       0, ...
        'meanProbot',      0, ...
        'meanMhuman',      1, ...
        'meanCollNurse',   0, ...
        'meanCollRobot',   0, ...
        'meanCollWall',    0, ...
        'meanCompleted',   0), ...
        numel(strategies), 1);

    for s = 1:numel(strategies)
        name   = strategies(s).name;
        mask   = strcmp({allResults.strategy}, name);
        subset = allResults(mask);

        T = subset(1).simDuration;
        ALPHA = 1.4;

        % Recompute C_task mean for display
        ctask_vals = zeros(numel(subset), 1);
        for si = 1:numel(subset)
            tau_c = subset(si).completionTimes / T;
            tau_u = ALPHA * (T - subset(si).uncompletedSpawnTimes) / T;
            all_t = [tau_c; tau_u];
            if isempty(all_t)
                ctask_vals(si) = 1.0;
            else
                ctask_vals(si) = mean(all_t);
            end
        end

        summary(s).strategy      = name;
        summary(s).meanScore     = mean([subset.score]);
        summary(s).meanCtask     = mean(ctask_vals);
        summary(s).meanPwall     = 0.042 * mean([subset.collisionsWall]);
        summary(s).meanProbot    = 0.170 * mean([subset.collisionsRobot]);
        summary(s).meanMhuman    = 1 + mean([subset.collisionsNurse]);
        summary(s).meanCollNurse = mean([subset.collisionsNurse]);
        summary(s).meanCollRobot = mean([subset.collisionsRobot]);
        summary(s).meanCollWall  = mean([subset.collisionsWall]);
        summary(s).meanCompleted = mean([subset.tasksCompleted]);
    end

    % Sort by mean score ascending
    scores = [summary.meanScore]';
    [~, ord] = sort(scores, 'ascend');

    fprintf('\n===== LEADERBOARD (lower score = better) =====\n');
    fprintf('  %3s  %-24s %7s   %7s  %7s  %7s  %7s   %6s  %6s\n', ...
        'Rank', 'Strategy', 'Score S', ...
        'C_task', 'P_wall', 'P_robot', 'M_human', ...
        'C_rn', 'done');
    fprintf('  %s\n', repmat('-', 1, 82));
    for rank = 1:numel(ord)
        s = summary(ord(rank));
        fprintf('  #%d   %-24s %7.4f = (%6.3f + %6.3f + %6.3f) x %5.2f  |  C_rn=%.1f  done=%.1f\n', ...
            rank, s.strategy, s.meanScore, ...
            s.meanCtask, s.meanPwall, s.meanProbot, ...
            s.meanMhuman, s.meanCollNurse, s.meanCompleted);
    end
    fprintf('  %s\n', repmat('=', 1, 82));

    % Highlight winner
    winner = summary(ord(1));
    fprintf('\n  Winner: %s  (S = %.4f)\n', winner.strategy, winner.meanScore);

    % Human safety flag
    for rank = 1:numel(ord)
        s = summary(ord(rank));
        if s.meanCollNurse >= 1
            fprintf('  [!] %s averaged %.1f nurse collision(s) — human safety concern\n', ...
                s.strategy, s.meanCollNurse);
        end
    end
    fprintf('\n');
end
```

---

## Step 5 — Update reproducibility check

The existing reproducibility check in `main.m` compares `collisions`, `tasksCompleted`,
and `avgTime`. Extend it to also check `score`:

```matlab
isDeterministic = probeA.collisions     == probeB.collisions     && ...
                  probeA.tasksCompleted == probeB.tasksCompleted  && ...
                  abs(probeA.avgTime    -  probeB.avgTime)  < 1e-10 && ...
                  abs(probeA.score      -  probeB.score)    < 1e-10;
```

---

## Summary of files changed

| File | Changes |
|------|---------|
| `HospitalEnv.m` | 3 new collision sub-counters; `detectCollisions` increments them |
| `main.m` | `allResults` struct extended; `computeScore` function added; score logged per run; `printLeaderboard` rewritten |

No changes to `RobotAgent.m`, `NurseAgent.m`, or `TaskDispatcher.m`.
