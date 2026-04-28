%% main.m  –  Hospital Multi-Agent Simulation Entry Point
%
%   Initialises all entities, wires up the environment, and runs the
%   main simulation loop at a fixed time step of dt = 0.1 seconds.
%
%   Configuration parameters are collected at the top of this file so
%   the user does not need to edit any class files to tune the scenario.
%
%   -----------------------------------------------------------------------
%   QUICK-START
%     1.  Place all .m files in the same folder (or on the MATLAB path).
%     2.  Run:  main
%   -----------------------------------------------------------------------

clear; close all; clc;

%% ======================== CONFIGURATION ================================ %%

% World (60 x 40 m double-loaded ward — see HospitalEnv.buildDefaultObstacles)
WORLD_W = 60;   % [m]
WORLD_H = 40;

% Simulation timing
dt          = 0.1;      % [s] fixed time step
SIM_DURATION = 120;     % [s] total run time (set Inf for endless)

% Robots
N_ROBOTS      = 4;      % number of autonomous delivery robots
ROBOT_RADIUS  = 0.40;   % [m]
LIDAR_RAYS    = 72;     % ray count  (multiple of 4 recommended)
LIDAR_FOV_DEG = 360;    % field of view [deg]
LIDAR_RANGE   = 8.0;    % [m]

% Nurses
N_NURSES      = 5;      % plain nurses
N_CART_NURSES = 2;      % nurses with carts (larger radius)

% Task dispatcher
TASK_MEAN_INTERVAL = 5.0;   % [s] average inter-arrival time
TASK_BATCH_PROB    = 0.10;  % probability of multi-task batch spawn

% Rendering
RENDER_EVERY = 2;   % render every N steps  (1 = every step, higher = faster)
SHOW_LIDAR   = true;
RUN_RENDER   = true; % false recommended for fast seeded benchmarking

% Runtime speed control
SPEED_MIN     = 0.25;  % minimum simulation speed [x]
SPEED_MAX     = 8.0;   % maximum simulation speed [x]
INITIAL_SPEED = 1.0;   % initial simulation speed [x]

%% ======================== BENCHMARK CONFIG ============================= %%

seedList = [11];
strategies = struct( ...
    'name', {'conservative_speed', 'aggressive_speed', 'dispatcher_round_robin'}, ...
    'controlMode', {'conservative_speed', 'aggressive_speed', 'default'}, ...
    'assignmentPolicy', {'nearest_idle', 'nearest_idle', 'round_robin'});

fprintf('Starting seeded benchmark (%d strategies x %d seeds)\n', ...
    numel(strategies), numel(seedList));

simCfg = struct( ...
    'WORLD_W', WORLD_W, ...
    'WORLD_H', WORLD_H, ...
    'SHOW_LIDAR', SHOW_LIDAR, ...
    'RUN_RENDER', RUN_RENDER, ...
    'INITIAL_SPEED', INITIAL_SPEED, ...
    'SPEED_MIN', SPEED_MIN, ...
    'SPEED_MAX', SPEED_MAX, ...
    'N_ROBOTS', N_ROBOTS, ...
    'ROBOT_RADIUS', ROBOT_RADIUS, ...
    'LIDAR_RAYS', LIDAR_RAYS, ...
    'LIDAR_FOV_DEG', LIDAR_FOV_DEG, ...
    'LIDAR_RANGE', LIDAR_RANGE, ...
    'N_NURSES', N_NURSES, ...
    'N_CART_NURSES', N_CART_NURSES, ...
    'TASK_MEAN_INTERVAL', TASK_MEAN_INTERVAL, ...
    'TASK_BATCH_PROB', TASK_BATCH_PROB, ...
    'dt', dt, ...
    'SIM_DURATION', SIM_DURATION, ...
    'RENDER_EVERY', RENDER_EVERY);

allResults = repmat(struct( ...
    'strategy', '', ...
    'seed', 0, ...
    'collisions', 0, ...
    'avgTime', Inf, ...
    'tasksCompleted', 0, ...
    'wallTime', 0), ...
    numel(strategies) * numel(seedList), 1);
resultIdx = 0;

for s = 1:numel(strategies)
    cfg = strategies(s);
    fprintf('\n=== Strategy: %s | control=%s | assignment=%s ===\n', ...
        cfg.name, cfg.controlMode, cfg.assignmentPolicy);
    for seed = seedList
        resultIdx = resultIdx + 1;
        runResult = runSingleExperiment(cfg, seed, simCfg);
        allResults(resultIdx) = runResult;
    end
end

printLeaderboard(allResults, strategies);

%% ======================== VALIDATION CHECKS ============================ %%

fprintf('\nRunning reproducibility check on first strategy/seed...\n');
probeCfg = strategies(1);
probeSeed = seedList(1);
probeA = runSingleExperiment(probeCfg, probeSeed, simCfg);
probeB = runSingleExperiment(probeCfg, probeSeed, simCfg);

isDeterministic = probeA.collisions == probeB.collisions && ...
    probeA.tasksCompleted == probeB.tasksCompleted && ...
    abs(probeA.avgTime - probeB.avgTime) < 1e-10;

if isDeterministic
    fprintf('Reproducibility check PASSED for strategy=%s seed=%d\n', ...
        probeCfg.name, probeSeed);
else
    warning('main:nonDeterministic', ...
        ['Reproducibility check FAILED for strategy=%s seed=%d. ', ...
         'Metrics differed between repeated runs.'], ...
        probeCfg.name, probeSeed);
end

%% ======================== HELPER FUNCTIONS ============================= %%

function positions = safeRandomPositions(n, env, minClearance, regionAABB)
    % Sample n collision-free positions inside the world (or inside regionAABB).
    % regionAABB (optional): [cx, cy, halfW, halfH] axis-aligned sampling box.
    positions = zeros(n, 2);
    margin    = minClearance + 0.5;
    W = env.Width; H = env.Height;

    useRegion = nargin >= 4 && ~isempty(regionAABB) && numel(regionAABB) == 4;
    if useRegion
        rcx = regionAABB(1); rcy = regionAABB(2);
        rhw = regionAABB(3); rhh = regionAABB(4);
        xMin = max(margin, rcx - rhw);
        xMax = min(W - margin, rcx + rhw);
        yMin = max(margin, rcy - rhh);
        yMax = min(H - margin, rcy + rhh);
        if xMax <= xMin || yMax <= yMin
            useRegion = false;
        end
    end

    placed = 0;
    maxAttempts = 5000;
    attempts    = 0;

    while placed < n && attempts < maxAttempts
        attempts = attempts + 1;
        if useRegion
            x = xMin + rand() * (xMax - xMin);
            y = yMin + rand() * (yMax - yMin);
        else
            x = margin + rand() * (W - 2*margin);
            y = margin + rand() * (H - 2*margin);
        end

        % Check against static obstacles (using env's private AABB logic
        % is not accessible; we replicate the key check here)
        clearOfObs = true;
        for w = 1:size(env.Obstacles,1)
            obs = env.Obstacles(w,:);
            dx = abs(x - obs(1)); dy = abs(y - obs(2));
            hw = obs(3);          hh = obs(4);
            if dx < hw + minClearance && dy < hh + minClearance
                clearOfObs = false; break;
            end
        end
        if ~clearOfObs, continue; end

        % Check against already-placed entities
        clearOfOthers = true;
        for j = 1:placed
            if norm([x - positions(j,1), y - positions(j,2)]) < 2*minClearance
                clearOfOthers = false; break;
            end
        end
        if ~clearOfOthers, continue; end

        placed = placed + 1;
        positions(placed,:) = [x, y];
    end

    if placed < n
        warning('main:placement', ...
            'Could only place %d/%d entities safely; world may be overcrowded.', ...
            placed, n);
    end
end

function result = runSingleExperiment(strategyCfg, seed, simCfg)
    % Build a fresh simulation instance for one (strategy, seed) pair.
    rng(seed, 'twister');

    fprintf('\n[RUN] strategy=%s seed=%d\n', strategyCfg.name, seed);
    env = HospitalEnv(simCfg.WORLD_W, simCfg.WORLD_H, strategyCfg.assignmentPolicy, simCfg.RUN_RENDER);
    env.setLidarVisible(simCfg.SHOW_LIDAR);
    if simCfg.RUN_RENDER
        env.addSpeedControl(simCfg.INITIAL_SPEED, simCfg.SPEED_MIN, simCfg.SPEED_MAX);
    end

    % Robots start in the main corridor strip (clear of room furniture).
    robotRegion = [30, 21, 28.5, 2.8];  % [cx, cy, halfW, halfH] in metres
    robotStartPositions = safeRandomPositions(simCfg.N_ROBOTS, env, ...
        simCfg.ROBOT_RADIUS + 0.5, robotRegion);
    for i = 1:simCfg.N_ROBOTS
        initTheta = rand() * 2 * pi;
        r = RobotAgent(robotStartPositions(i,1), robotStartPositions(i,2), ...
            initTheta, simCfg.LIDAR_RAYS, simCfg.LIDAR_FOV_DEG, simCfg.LIDAR_RANGE);
        r.setControlMode(strategyCfg.controlMode);
        env.addRobot(r);
    end

    % Nurses: corridor + lobby / clinical circulation (avoids spawning inside beds).
    nN = simCfg.N_NURSES + simCfg.N_CART_NURSES;
    nCor = ceil(0.55 * nN);
    nLob = nN - nCor;
    nurseCorridor = safeRandomPositions(nCor, env, 0.6, [30, 21, 28.5, 2.8]);
    nurseLobby = safeRandomPositions(nLob, env, 0.6, [56, 10, 2.8, 8.5]);
    nursePositions = [nurseCorridor; nurseLobby];
    nursePositions = nursePositions(randperm(nN), :);
    for k = 1:simCfg.N_NURSES
        env.addNurse(NurseAgent(nursePositions(k,1), nursePositions(k,2), false));
    end
    for k = simCfg.N_NURSES+1 : simCfg.N_NURSES+simCfg.N_CART_NURSES
        env.addNurse(NurseAgent(nursePositions(k,1), nursePositions(k,2), true));
    end

    dispatcher = TaskDispatcher(env, simCfg.TASK_MEAN_INTERVAL, simCfg.TASK_BATCH_PROB);

    stepCount = 0;
    renderCount = 0;
    runTimer = tic;
    while env.SimTime < simCfg.SIM_DURATION
        dispatcher.update(simCfg.dt);
        env.step(simCfg.dt);
        stepCount = stepCount + 1;
        renderCount = renderCount + 1;

        if simCfg.RUN_RENDER && renderCount >= simCfg.RENDER_EVERY
            env.render();
            renderCount = 0;
        end

        if simCfg.RUN_RENDER && ~env.isFigureOpen()
            break;
        end
    end

    wallTime = toc(runTimer);
    if isempty(env.CompletionTimes)
        avgTime = Inf;
    else
        avgTime = mean(env.CompletionTimes);
    end

    result.strategy = strategyCfg.name;
    result.seed = seed;
    result.collisions = env.TotalCollisions;
    result.avgTime = avgTime;
    result.tasksCompleted = numel(env.CompletionTimes);
    result.wallTime = wallTime;

    fprintf('[RESULT] collisions=%d avgTime=%.3f completed=%d wall=%.2fs steps=%d\n', ...
        result.collisions, result.avgTime, result.tasksCompleted, result.wallTime, stepCount);
end

function printLeaderboard(allResults, strategies)
    fprintf('\n===== PER-SEED RESULTS =====\n');
    for i = 1:numel(allResults)
        rr = allResults(i);
        fprintf('  %-24s seed=%3d collisions=%4d avgTime=%8.3f completed=%3d\n', ...
            rr.strategy, rr.seed, rr.collisions, rr.avgTime, rr.tasksCompleted);
    end

    summary = repmat(struct('strategy','','meanCollisions',0,'meanAvgTime',Inf,'meanCompleted',0), ...
        numel(strategies), 1);
    for s = 1:numel(strategies)
        name = strategies(s).name;
        mask = strcmp({allResults.strategy}, name);
        subset = allResults(mask);
        summary(s).strategy = name;
        summary(s).meanCollisions = mean([subset.collisions]);
        summary(s).meanAvgTime = mean([subset.avgTime]);
        summary(s).meanCompleted = mean([subset.tasksCompleted]);
    end

    collisions = [summary.meanCollisions]';
    avgTimes = [summary.meanAvgTime]';
    rankingTable = table((1:numel(summary))', collisions, avgTimes, ...
        'VariableNames', {'idx','collisions','avgTime'});
    rankingTable = sortrows(rankingTable, {'collisions','avgTime'}, {'ascend','ascend'});

    fprintf('\n===== LEADERBOARD (collisions first, avg time tie-break) =====\n');
    for rank = 1:height(rankingTable)
        i = rankingTable.idx(rank);
        fprintf('  #%d %-24s collisions=%.2f avgTime=%.3f avgCompleted=%.2f\n', ...
            rank, summary(i).strategy, summary(i).meanCollisions, ...
            summary(i).meanAvgTime, summary(i).meanCompleted);
    end
    fprintf('==============================================================\n');
end
