classdef RobotAgent < handle
    % RobotAgent  Autonomous hospital delivery robot.
    %
    %   Each instance represents one physical robot with:
    %     - Unicycle kinematic model  (x, y, theta)
    %     - 2-D simulated Lidar sensor
    %     - Pluggable velocity controller (override calculateVelocity)
    %
    %   The robot has NO global knowledge.  It only perceives:
    %     - Its own pose  [X, Y, Theta]
    %     - Its current target  [TargetX, TargetY]
    %     - Its local Lidar scan  LidarRanges / LidarAngles

    % ------------------------------------------------------------------ %
    properties (SetAccess = private)
        % Pose
        X      (1,1) double = 0
        Y      (1,1) double = 0
        Theta  (1,1) double = 0   % [rad]  heading angle

        % Physical footprint
        Radius (1,1) double = 0.4  % [m] locker-chassis radius

        % Current navigation target
        TargetX       (1,1) double = NaN
        TargetY       (1,1) double = NaN
        CurrentTaskID (1,1) double = -1

        % Robot status  'idle' | 'busy' | 'error' | 'out_of_operation'
        Status (1,:) char = 'idle'
        ControlMode (1,:) char = 'default'

        % Lidar configuration (read once, used every step)
        NumRays    (1,1) double = 72     % ray count
        FoVRad     (1,1) double = 2*pi   % field-of-view [rad]  (2π = 360°)
        MaxRange   (1,1) double = 8.0    % [m]
        LidarNoise (1,1) double = 0.02   % Gaussian σ [m]

        % Lidar output (updated each sense step)
        LidarRanges (:,1) double   % [m]  one per ray
        LidarAngles (:,1) double   % [rad] local angle per ray (relative to robot)

        % Velocity limits
        MaxLinSpeed  (1,1) double = 1.0   % [m/s]
        MaxAngSpeed  (1,1) double = pi    % [rad/s]

        % Command smoothing / rate limits
        MaxLinAccel  (1,1) double = 2.5   % [m/s^2]
        MaxAngAccel  (1,1) double = 6.0   % [rad/s^2]
        LastV        (1,1) double = 0.0
        LastOmega    (1,1) double = 0.0

        % Optional debug logging (disabled by default for runtime performance)
        DebugEnabled (1,1) logical = false

        % Global planning state (known static geometry + path cursor)
        KnownMapWidth  (1,1) double = NaN
        KnownMapHeight (1,1) double = NaN
        KnownObstacles (:,4) double = zeros(0,4)
        HasKnownMap (1,1) logical = false
        PlannedWaypoints (:,2) double = zeros(0,2)
        WaypointIndex (1,1) double = 1
        LastPlannedTarget (1,2) double = [NaN NaN]
        LastPlanTime (1,1) double = -Inf
        SimTimeCache (1,1) double = 0
        DtCache (1,1) double = 0.1

        % Planner tuning
        GridResolution (1,1) double = 0.6
        ReplanInterval (1,1) double = 0.7
        WaypointTolerance (1,1) double = 0.55
        NurseSafetyMargin (1,1) double = 0.95
        NurseYieldStartDist (1,1) double = 2.0
        NurseYieldStopDist  (1,1) double = 1.05
        MaxYieldHoldTime    (1,1) double = 0.9
        YieldHoldTime       (1,1) double = 0.0
        PrevFrontRange      (1,1) double = NaN
        ExternalHoldUntil   (1,1) double = -Inf
        LastProgressDistance (1,1) double = Inf
        LastProgressTime    (1,1) double = 0
        ReverseUntil        (1,1) double = -Inf
        DeadlockBoostUntil  (1,1) double = -Inf
    end

    % ------------------------------------------------------------------ %
    methods
        % ============================================================== %
        function obj = RobotAgent(x, y, theta, numRays, fovDeg, maxRange)
            obj.X     = x;
            obj.Y     = y;
            obj.Theta = theta;

            if nargin >= 4, obj.NumRays  = numRays;          end
            if nargin >= 5, obj.FoVRad   = deg2rad(fovDeg);  end
            if nargin >= 6, obj.MaxRange = maxRange;          end

            % Pre-compute fixed ray angles (local frame, symmetric about 0)
            halfFoV = obj.FoVRad / 2;
            obj.LidarAngles = linspace(-halfFoV, halfFoV, obj.NumRays)';
            obj.LidarRanges = obj.MaxRange * ones(obj.NumRays, 1);
        end

        % ============================================================== %
        function assignTarget(obj, tx, ty, taskID)
            obj.TargetX       = tx;
            obj.TargetY       = ty;
            obj.CurrentTaskID = taskID;
            obj.Status        = 'busy';
        end

        function clearTarget(obj)
            obj.TargetX       = NaN;
            obj.TargetY       = NaN;
            obj.CurrentTaskID = -1;
            obj.LastProgressDistance = Inf;
            obj.ReverseUntil = -Inf;
            obj.DeadlockBoostUntil = -Inf;
            obj.ExternalHoldUntil = -Inf;
            if ~strcmp(obj.Status, 'out_of_operation')
                obj.Status = 'idle';
            end
        end

        function retire(obj)
            obj.clearTarget();
            obj.Status = 'out_of_operation';
            obj.LastV = 0;
            obj.LastOmega = 0;
        end

        function tf = isOperational(obj)
            tf = ~strcmp(obj.Status, 'out_of_operation');
        end

        function state = getDebugState(obj)
            % Compact label of the dominant runtime constraint, used by
            % HospitalEnv to render an above-chassis status badge.
            if ~obj.isOperational()
                state = 'down';
                return;
            end
            if obj.SimTimeCache < obj.ReverseUntil
                state = 'reversing';
                return;
            end
            if obj.SimTimeCache < obj.ExternalHoldUntil
                state = 'holding';
                return;
            end
            if obj.YieldHoldTime > 0
                state = 'yielding';
                return;
            end
            if obj.SimTimeCache < obj.DeadlockBoostUntil
                state = 'boosted';
                return;
            end
            if strcmp(obj.Status, 'idle')
                state = 'idle';
                return;
            end
            state = 'cruising';
        end

        function setControlMode(obj, mode)
            allowed = {'default', 'conservative_speed', 'aggressive_speed'};
            if ~any(strcmp(mode, allowed))
                error('RobotAgent:InvalidControlMode', ...
                    'Unsupported control mode: %s', mode);
            end
            obj.ControlMode = mode;
        end

        function setStaticMap(obj, mapData)
            obj.KnownMapWidth = mapData.width;
            obj.KnownMapHeight = mapData.height;
            obj.KnownObstacles = mapData.obstacles;
            obj.HasKnownMap = true;
        end

        function configurePlanner(obj, cfg)
            if isfield(cfg, 'gridResolution')
                obj.GridResolution = cfg.gridResolution;
            end
            if isfield(cfg, 'replanInterval')
                obj.ReplanInterval = cfg.replanInterval;
            end
            if isfield(cfg, 'waypointTolerance')
                obj.WaypointTolerance = cfg.waypointTolerance;
            end
            if isfield(cfg, 'nurseSafetyMargin')
                obj.NurseSafetyMargin = cfg.nurseSafetyMargin;
            end
            if isfield(cfg, 'nurseYieldStartDist')
                obj.NurseYieldStartDist = cfg.nurseYieldStartDist;
            end
            if isfield(cfg, 'nurseYieldStopDist')
                obj.NurseYieldStopDist = cfg.nurseYieldStopDist;
            end
            if isfield(cfg, 'maxYieldHoldTime')
                obj.MaxYieldHoldTime = cfg.maxYieldHoldTime;
            end
        end

        function updatePlanningContext(obj, simTime, dt)
            obj.SimTimeCache = simTime;
            obj.DtCache = dt;
        end

        function setExternalHoldUntil(obj, holdUntil)
            obj.ExternalHoldUntil = max(obj.ExternalHoldUntil, holdUntil);
        end

        function requestDeadlockRecovery(obj, boostDuration)
            if nargin < 2 || isempty(boostDuration)
                boostDuration = 0.5;
            end
            obj.DeadlockBoostUntil = max(obj.DeadlockBoostUntil, obj.SimTimeCache + boostDuration);
            obj.ReverseUntil = max(obj.ReverseUntil, obj.SimTimeCache + 0.35);
        end

        % ============================================================== %
        function lidarData = senseEnvironment(obj, obstacles, nurses, otherRobots)
            % senseEnvironment  Cast all Lidar rays; return range array.
            %
            %   Geometry cast against:
            %     1. AABB obstacles  (walls / structural blocks)
            %     2. Circular agents (nurses, other robots)
            %
            %   Returns
            %   -------
            %   lidarData  struct with fields:
            %     .ranges  [NumRays x 1]  range to nearest hit [m]
            %     .angles  [NumRays x 1]  ray angles in local frame [rad]

            ranges = obj.MaxRange * ones(obj.NumRays, 1);

            % World-frame ray directions
            worldAngles = obj.LidarAngles + obj.Theta;
            dx = cos(worldAngles);   % [NumRays x 1]
            dy = sin(worldAngles);

            % --- 1. AABB slab intersections (vectorised over obstacles) ---
            for w = 1:size(obstacles,1)
                cx = obstacles(w,1); cy = obstacles(w,2);
                hw = obstacles(w,3); hh = obstacles(w,4);
                t = obj.rayAABBIntersect(obj.X, obj.Y, dx, dy, cx, cy, hw, hh);
                ranges = min(ranges, t);
            end

            % --- 2. Circle intersections (nurses + other robots) ----------
            % Iterate each homogeneous class array separately.
            for k = 1:numel(nurses)
                e  = nurses(k);
                ox = e.X - obj.X;
                oy = e.Y - obj.Y;
                t  = obj.rayCircleIntersect(ox, oy, e.Radius, dx, dy);
                ranges = min(ranges, t);
            end
            for k = 1:numel(otherRobots)
                e  = otherRobots(k);
                ox = e.X - obj.X;
                oy = e.Y - obj.Y;
                t  = obj.rayCircleIntersect(ox, oy, e.Radius, dx, dy);
                ranges = min(ranges, t);
            end

            % --- 3. Clamp & add Gaussian noise ----------------------------
            ranges = max(0, min(obj.MaxRange, ranges));
            ranges = ranges + obj.LidarNoise * randn(obj.NumRays, 1);
            ranges = max(0, min(obj.MaxRange, ranges));

            obj.LidarRanges = ranges;

            lidarData.ranges = ranges;
            lidarData.angles = obj.LidarAngles;   % local frame
        end

        % ============================================================== %
        function [v, omega] = calculateVelocity(obj, target_x, target_y, lidar_data) %#ok<INUSD>
            % calculateVelocity  Default controller – override with your strategy.
            %
            %   Default behaviour:
            %     • No target  →  [0, 0]
            %     • Has target →  proportional heading + constant forward speed
            %
            %   Parameters (for your override)
            %   ---------------------------------
            %   target_x, target_y  – goal position in world frame [m]
            %   lidar_data          – struct from senseEnvironment()
            %
            %   Returns
            %   -------
            %   v      linear velocity  [m/s]
            %   omega  angular velocity [rad/s]

            if ~obj.isOperational()
                v = 0; omega = 0;
                return;
            end

            if isnan(target_x) || isnan(target_y)
                v = 0; omega = 0;
                obj.PlannedWaypoints = zeros(0,2);
                obj.WaypointIndex = 1;
                obj.LastProgressDistance = Inf;
                return;
            end

            if obj.SimTimeCache < obj.ExternalHoldUntil
                v = 0; omega = 0;
                return;
            end

            if obj.HasKnownMap
                mustReplan = obj.needsReplan(target_x, target_y);
                if mustReplan
                    ok = obj.planPathToTarget(target_x, target_y);
                    if ~ok
                        v = 0; omega = 0;
                        return;
                    end
                end
            else
                obj.PlannedWaypoints = [target_x, target_y];
                obj.WaypointIndex = 1;
            end

            [wx, wy, hasWp] = obj.getCurrentWaypoint(target_x, target_y);
            if ~hasWp
                v = 0; omega = 0;
                return;
            end

            dx = wx - obj.X;
            dy = wy - obj.Y;
            distWp = norm([dx, dy]);
            goalDist = norm([target_x - obj.X, target_y - obj.Y]);
            if distWp < obj.WaypointTolerance
                obj.WaypointIndex = obj.WaypointIndex + 1;
                [wx, wy, hasWp] = obj.getCurrentWaypoint(target_x, target_y);
                if ~hasWp
                    v = 0; omega = 0;
                    return;
                end
                dx = wx - obj.X;
                dy = wy - obj.Y;
                distWp = norm([dx, dy]);
            end

            targetAngle = atan2(dy, dx);
            headingErr = wrapToPi(targetAngle - obj.Theta);

            [isBlocked, canRetryPlan] = obj.checkLocalBlockage(lidar_data);
            if isBlocked
                if canRetryPlan
                    ok = obj.planPathToTarget(target_x, target_y);
                    if ok
                        [wx2, wy2, hasWp2] = obj.getCurrentWaypoint(target_x, target_y);
                        if hasWp2
                            dx = wx2 - obj.X;
                            dy = wy2 - obj.Y;
                            targetAngle = atan2(dy, dx);
                            headingErr = wrapToPi(targetAngle - obj.Theta);
                            [isBlocked2, ~] = obj.checkLocalBlockage(lidar_data);
                            if isBlocked2
                                v = 0; omega = 0;
                                return;
                            end
                        end
                    else
                        v = 0; omega = 0;
                        return;
                    end
                else
                    v = 0; omega = 0;
                    return;
                end
            end

            if goalDist + 0.03 < obj.LastProgressDistance
                obj.LastProgressDistance = goalDist;
                obj.LastProgressTime = obj.SimTimeCache;
            elseif (obj.SimTimeCache - obj.LastProgressTime) > 1.2
                obj.requestDeadlockRecovery(0.6);
                obj.LastProgressTime = obj.SimTimeCache;
            end

            switch obj.ControlMode
                case 'conservative_speed'
                    Kp_omega = 1.8;
                    speedScale = 0.6;
                case 'aggressive_speed'
                    Kp_omega = 2.4;
                    speedScale = 0.95;
                otherwise
                    Kp_omega = 2.0;
                    speedScale = 0.8;
            end

            alignFactor = max(0.25, cos(headingErr));
            distGain = min(1.0, distWp / 1.8);
            v = obj.MaxLinSpeed * speedScale * alignFactor * distGain;
            omega = Kp_omega * headingErr;

            [yieldScale, mustHold] = obj.assessNurseYield(lidar_data);
            if mustHold
                v = 0;
                omega = 0;
            else
                v = v * yieldScale;
            end

            [v, omega] = obj.applyReciprocalAvoidance(v, omega, headingErr, lidar_data);

            if obj.SimTimeCache < obj.ReverseUntil
                v = -0.18;
                omega = 0;
            elseif obj.SimTimeCache < obj.DeadlockBoostUntil
                v = min(obj.MaxLinSpeed, v * 1.12);
            end

            omega = max(-obj.MaxAngSpeed, min(obj.MaxAngSpeed, omega));
            if v <= 1e-3
                % No point turns unless coupled with forward movement.
                omega = 0;
            end
        end

        % ============================================================== %
        function move(obj, v, omega, dt, obstacles, worldW, worldH)
            % move  Integrate unicycle kinematics; enforce world bounds.
            if ~obj.isOperational()
                obj.LastV = 0;
                obj.LastOmega = 0;
                return;
            end
            [v, omega] = obj.applyCommandSmoothing(v, omega, dt);

            % Unicycle integration
            newTheta = obj.Theta + omega * dt;
            newX     = obj.X + v * cos(obj.Theta) * dt;
            newY     = obj.Y + v * sin(obj.Theta) * dt;

            % Clamp to world bounds (with radius padding)
            newX = max(obj.Radius, min(worldW - obj.Radius, newX));
            newY = max(obj.Radius, min(worldH - obj.Radius, newY));

            % AABB push-out for static obstacles
            [newX, newY] = obj.resolveObstacleCollision(newX, newY, obstacles);

            obj.X     = newX;
            obj.Y     = newY;
            obj.Theta = wrapToPi(newTheta);
        end
    end

    % ------------------------------------------------------------------ %
    methods (Access = private, Static)
        function appendDebugLog(payload)
            fid = fopen('debug-9db484.log','a');
            if fid == -1
                return;
            end
            fprintf(fid, '%s\n', jsonencode(payload));
            fclose(fid);
        end
    end

    % ------------------------------------------------------------------ %
    methods (Access = private)

        function tf = needsReplan(obj, targetX, targetY)
            targetChanged = any(abs([targetX, targetY] - obj.LastPlannedTarget) > 0.25);
            pathMissing = isempty(obj.PlannedWaypoints) || obj.WaypointIndex > size(obj.PlannedWaypoints,1);
            stale = (obj.SimTimeCache - obj.LastPlanTime) >= obj.ReplanInterval;
            tf = targetChanged || pathMissing || stale;
        end

        function ok = planPathToTarget(obj, targetX, targetY)
            [path, ok] = obj.aStarPath([obj.X, obj.Y], [targetX, targetY]);
            if ok
                obj.PlannedWaypoints = path;
                obj.WaypointIndex = 1;
                obj.LastPlannedTarget = [targetX, targetY];
                obj.LastPlanTime = obj.SimTimeCache;
            else
                obj.PlannedWaypoints = zeros(0,2);
                obj.WaypointIndex = 1;
            end
        end

        function [wx, wy, ok] = getCurrentWaypoint(obj, targetX, targetY)
            if isempty(obj.PlannedWaypoints)
                wx = targetX; wy = targetY; ok = true;
                return;
            end
            idx = min(obj.WaypointIndex, size(obj.PlannedWaypoints,1));
            wx = obj.PlannedWaypoints(idx,1);
            wy = obj.PlannedWaypoints(idx,2);
            ok = true;
        end

        function [blocked, canRetryPlan] = checkLocalBlockage(obj, lidarData)
            ang = lidarData.angles;
            rngs = lidarData.ranges;
            frontMask = abs(ang) <= deg2rad(25);
            frontClear = min(rngs(frontMask));
            blocked = frontClear < obj.NurseSafetyMargin;
            canRetryPlan = blocked && ((obj.SimTimeCache - obj.LastPlanTime) >= obj.ReplanInterval);
        end

        function [yieldScale, mustHold] = assessNurseYield(obj, lidarData)
            % Balanced nurse-priority yielding using only local lidar cues.
            ang = lidarData.angles;
            rngs = lidarData.ranges;
            frontMask = abs(ang) <= deg2rad(28);
            crossingMask = abs(ang) > deg2rad(18) & abs(ang) <= deg2rad(45);

            frontR = min(rngs(frontMask));
            crossR = min(rngs(crossingMask));
            if isempty(frontR) || ~isfinite(frontR)
                frontR = obj.MaxRange;
            end
            if isempty(crossR) || ~isfinite(crossR)
                crossR = obj.MaxRange;
            end

            closingRate = 0;
            if isfinite(obj.PrevFrontRange) && obj.DtCache > 1e-6
                closingRate = (obj.PrevFrontRange - frontR) / obj.DtCache;
            end
            obj.PrevFrontRange = frontR;

            mustHold = false;
            if frontR <= obj.NurseYieldStopDist || ...
                    (frontR <= obj.NurseYieldStartDist && crossR <= obj.NurseYieldStartDist && closingRate > 0.2)
                obj.YieldHoldTime = obj.MaxYieldHoldTime;
            else
                obj.YieldHoldTime = max(0, obj.YieldHoldTime - obj.DtCache);
            end
            if obj.YieldHoldTime > 0
                mustHold = true;
                yieldScale = 0;
                return;
            end

            if frontR <= obj.NurseYieldStopDist
                yieldScale = 0;
                mustHold = true;
                return;
            end
            if frontR >= obj.NurseYieldStartDist
                yieldScale = 1.0;
            else
                span = max(1e-6, obj.NurseYieldStartDist - obj.NurseYieldStopDist);
                alpha = (frontR - obj.NurseYieldStopDist) / span;
                yieldScale = max(0.2, min(1.0, alpha));
            end
            if crossR < obj.NurseYieldStartDist
                yieldScale = min(yieldScale, 0.55);
            end
        end

        function [vOut, omegaOut] = applyReciprocalAvoidance(obj, vIn, omegaIn, headingErr, lidarData)
            % Lightweight reciprocal-avoidance proxy from local scan sectors.
            ang = lidarData.angles;
            rngs = lidarData.ranges;
            leftMask = ang > deg2rad(12) & ang <= deg2rad(65);
            rightMask = ang < -deg2rad(12) & ang >= -deg2rad(65);
            frontMask = abs(ang) <= deg2rad(22);
            leftClear = min(rngs(leftMask));
            rightClear = min(rngs(rightMask));
            frontClear = min(rngs(frontMask));
            if isempty(leftClear) || ~isfinite(leftClear), leftClear = obj.MaxRange; end
            if isempty(rightClear) || ~isfinite(rightClear), rightClear = obj.MaxRange; end
            if isempty(frontClear) || ~isfinite(frontClear), frontClear = obj.MaxRange; end

            sideBias = max(-1.0, min(1.0, (leftClear - rightClear) / max(obj.MaxRange, 1e-6)));
            omegaBias = -0.55 * sideBias;
            slowScale = min(1.0, max(0.18, frontClear / max(1.6, obj.NurseSafetyMargin + 0.5)));
            if abs(headingErr) < deg2rad(25)
                vOut = vIn * slowScale;
            else
                vOut = vIn * min(1.0, slowScale + 0.15);
            end
            omegaOut = omegaIn + omegaBias;
        end

        function [waypoints, ok] = aStarPath(obj, startXY, goalXY)
            % Grid-based A* over known static map.
            if ~obj.HasKnownMap
                waypoints = goalXY;
                ok = true;
                return;
            end

            res = obj.GridResolution;
            W = obj.KnownMapWidth;
            H = obj.KnownMapHeight;
            nx = max(2, floor(W / res) + 1);
            ny = max(2, floor(H / res) + 1);

            occ = false(ny, nx);
            inflate = obj.Radius + 0.12;
            [Xg, Yg] = meshgrid(0:(nx-1), 0:(ny-1));
            worldX = Xg * res;
            worldY = Yg * res;
            for k = 1:size(obj.KnownObstacles,1)
                o = obj.KnownObstacles(k,:);
                occ = occ | obj.gridCircleAABBOverlap(worldX, worldY, inflate, o);
            end

            [sx, sy] = obj.worldToGrid(startXY(1), startXY(2), res, nx, ny);
            [gx, gy] = obj.worldToGrid(goalXY(1), goalXY(2), res, nx, ny);
            if occ(sy, sx) || occ(gy, gx)
                waypoints = zeros(0,2);
                ok = false;
                return;
            end

            nNodes = nx * ny;
            startIdx = sub2ind([ny, nx], sy, sx);
            goalIdx  = sub2ind([ny, nx], gy, gx);
            open = false(nNodes,1);
            closed = false(nNodes,1);
            gScore = inf(nNodes,1);
            fScore = inf(nNodes,1);
            cameFrom = zeros(nNodes,1);

            gScore(startIdx) = 0;
            fScore(startIdx) = hypot(gx-sx, gy-sy);
            open(startIdx) = true;

            neigh = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
            while any(open)
                openIdx = find(open);
                [~, m] = min(fScore(openIdx));
                cur = openIdx(m);
                if cur == goalIdx
                    break;
                end
                open(cur) = false;
                closed(cur) = true;
                [cy, cx] = ind2sub([ny, nx], cur);

                for ni = 1:size(neigh,1)
                    nxCell = cx + neigh(ni,1);
                    nyCell = cy + neigh(ni,2);
                    if nxCell < 1 || nxCell > nx || nyCell < 1 || nyCell > ny
                        continue;
                    end
                    if occ(nyCell, nxCell)
                        continue;
                    end
                    nIdx = sub2ind([ny, nx], nyCell, nxCell);
                    if closed(nIdx)
                        continue;
                    end
                    stepCost = hypot(neigh(ni,1), neigh(ni,2));
                    tentative = gScore(cur) + stepCost;
                    if tentative < gScore(nIdx)
                        cameFrom(nIdx) = cur;
                        gScore(nIdx) = tentative;
                        fScore(nIdx) = tentative + hypot(gx - nxCell, gy - nyCell);
                        open(nIdx) = true;
                    end
                end
            end

            if ~isfinite(gScore(goalIdx))
                waypoints = zeros(0,2);
                ok = false;
                return;
            end

            chain = goalIdx;
            while chain(1) ~= startIdx
                prev = cameFrom(chain(1));
                if prev == 0
                    waypoints = zeros(0,2);
                    ok = false;
                    return;
                end
                chain = [prev; chain]; %#ok<AGROW>
            end

            waypoints = zeros(numel(chain), 2);
            for ii = 1:numel(chain)
                [gyi, gxi] = ind2sub([ny, nx], chain(ii));
                waypoints(ii,:) = [(gxi-1) * res, (gyi-1) * res];
            end
            waypoints(end,:) = goalXY;
            ok = true;
        end

        function [gx, gy] = worldToGrid(~, x, y, res, nx, ny)
            gx = min(nx, max(1, round(x / res) + 1));
            gy = min(ny, max(1, round(y / res) + 1));
        end

        function tf = gridCircleAABBOverlap(~, x, y, r, aabb)
            dx = abs(x - aabb(1));
            dy = abs(y - aabb(2));
            hw = aabb(3);
            hh = aabb(4);
            tf = ~(dx > hw + r | dy > hh + r);
            inside = (dx <= hw | dy <= hh);
            corner = (dx - hw).^2 + (dy - hh).^2 <= r^2;
            tf = tf & (inside | corner);
        end

        % ============================================================== %
        function [vCmd, omegaCmd] = applyCommandSmoothing(obj, vTarget, omegaTarget, dt)
            % Limit acceleration and angular acceleration for smoother motion.
            maxDv = obj.MaxLinAccel * dt;
            maxDw = obj.MaxAngAccel * dt;

            dv = max(-maxDv, min(maxDv, vTarget - obj.LastV));
            dw = max(-maxDw, min(maxDw, omegaTarget - obj.LastOmega));

            vCmd = obj.LastV + dv;
            omegaCmd = obj.LastOmega + dw;

            % Keep final commands inside hard velocity limits.
            vCmd = max(-obj.MaxLinSpeed, min(obj.MaxLinSpeed, vCmd));
            omegaCmd = max(-obj.MaxAngSpeed, min(obj.MaxAngSpeed, omegaCmd));

            obj.LastV = vCmd;
            obj.LastOmega = omegaCmd;
        end

        % ============================================================== %
        function t = rayAABBIntersect(~, ox, oy, dx, dy, cx, cy, hw, hh)
            % Slab method – vectorised over NumRays.
            % Returns intersection distance per ray (inf if no hit).
            xmin = cx - hw; xmax = cx + hw;
            ymin = cy - hh; ymax = cy + hh;

            % Shift origin
            relX = ox - cx; relY = oy - cy;

            % x-slabs
            with_dx = dx ~= 0;
            tx1 = inf(size(dx)); tx2 = inf(size(dx));
            tx1(with_dx) = (xmin - ox) ./ dx(with_dx);
            tx2(with_dx) = (xmax - ox) ./ dx(with_dx);
            txNear = min(tx1, tx2);
            txFar  = max(tx1, tx2);

            % y-slabs
            with_dy = dy ~= 0;
            ty1 = inf(size(dy)); ty2 = inf(size(dy));
            ty1(with_dy) = (ymin - oy) ./ dy(with_dy);
            ty2(with_dy) = (ymax - oy) ./ dy(with_dy);
            tyNear = min(ty1, ty2);
            tyFar  = max(ty1, ty2);

            tNear = max(txNear, tyNear);
            tFar  = min(txFar,  tyFar);

            hit = (tFar >= 0) & (tNear <= tFar) & (tNear >= 0);
            t   = inf(size(dx));
            t(hit) = tNear(hit);
        end

        % ============================================================== %
        function t = rayCircleIntersect(~, ox, oy, r, dx, dy)
            % Analytic ray-circle intersection (vectorised over NumRays).
            % ox,oy: vector from ray origin to circle centre.
            a  = dx.^2 + dy.^2;           % should be 1 for unit dirs
            b  = -2*(dx*ox + dy*oy);
            c  = ox^2 + oy^2 - r^2;
            discriminant = b.^2 - 4*a*c;

            t = inf(size(dx));
            valid = discriminant >= 0;
            if any(valid)
                sqrtD = sqrt(discriminant(valid));
                t1 = (-b(valid) - sqrtD) ./ (2*a(valid));
                t2 = (-b(valid) + sqrtD) ./ (2*a(valid));
                tHit = min(t1, t2);
                tHit(tHit < 0) = max(t1(tHit<0), t2(tHit<0));
                tHit(tHit < 0) = inf;
                t(valid) = tHit;
            end
        end

        % ============================================================== %
        function [nx, ny] = resolveObstacleCollision(obj, nx, ny, obstacles)
            % Simple AABB push-out: move robot to nearest surface if overlapping.
            for w = 1:size(obstacles,1)
                cx = obstacles(w,1); cy = obstacles(w,2);
                hw = obstacles(w,3); hh = obstacles(w,4);

                % Closest point on AABB to circle centre
                cpx = max(cx-hw, min(cx+hw, nx));
                cpy = max(cy-hh, min(cy+hh, ny));
                dist = norm([nx-cpx, ny-cpy]);

                if dist < obj.Radius && dist > 0
                    % Push out along penetration normal
                    pushDir = ([nx, ny] - [cpx, cpy]) / dist;
                    overlap = obj.Radius - dist;
                    nx = nx + pushDir(1) * overlap;
                    ny = ny + pushDir(2) * overlap;
                elseif dist == 0
                    % Centre is exactly on boundary edge – push straight out
                    nx = nx + obj.Radius;
                end
            end
        end
    end
end
