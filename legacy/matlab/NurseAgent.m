classdef NurseAgent < handle
    % NurseAgent  Moving obstacle representing a hospital nurse (±cart).
    %
    %   Nurses perform a random-waypoint walk:
    %     1. Pick a random free waypoint.
    %     2. Walk toward it at a randomly sampled speed.
    %     3. Pause at the waypoint for a random dwell time.
    %     4. Repeat.
    %
    %   Nurses have simple wall-bounce logic to stay inside the environment.
    %   They do NOT interact with each other or with task assignments.

    % ------------------------------------------------------------------ %
    properties
        % Optional semantic destinations (Nx2 [x y]) from HospitalEnv POIs.
        % When non-empty, waypoints are sampled as internal nurse intents.
        Waypoints (:,:) double = zeros(0,2)
    end

    properties (SetAccess = private)
        % Pose  (nurse is modelled as a circle; carts as a larger circle)
        X      (1,1) double
        Y      (1,1) double
        Theta  (1,1) double = 0   % heading [rad]

        % Physical footprint (kept constant for visual semantics)
        Radius  (1,1) double = 0.25   % [m]
        HasCart (1,1) logical = false

        % Internal navigation state (not exposed to robots)
        GoalX       (1,1) double
        GoalY       (1,1) double
        GoalType    (1,:) char = 'corridor'
        Speed       (1,1) double = 0      % [m/s] current speed
        DesiredSpeed (1,1) double = 0     % [m/s] mode target speed
        DwellTime   (1,1) double = 0      % [s]  remaining pause time
        AvoidTime   (1,1) double = 0      % [s]  temporary avoidance mode time
        State       (1,:) char = 'selecting_goal'  % selecting_goal | cruising | avoidance | dwelling
    end

    properties (Constant, Access = private)
        % Nurse behaviour parameters
        MinCruiseSpeed = 0.45    % [m/s]
        MaxCruiseSpeed = 1.20
        AvoidSpeedScale = 0.72
        MinDwell = 0.8           % [s]
        MaxDwell = 4.0
        WpMargin = 1.8           % [m] keep goals away from world edges
        GoalReachTol = 0.28      % [m]
        MaxAccel = 1.8           % [m/s^2]
        MaxTurnRate = 2.8        % [rad/s]
        ObstacleSense = 1.35     % [m]
        NeighborSense = 1.10     % [m]
        EdgeSense = 0.95         % [m]
        RadiusNurse = 0.25       % [m]
    end

    % ------------------------------------------------------------------ %
    methods
        % ============================================================== %
        function obj = NurseAgent(x, y, hasCart)
            obj.X = x;
            obj.Y = y;
            if nargin >= 3 && hasCart
                obj.HasCart = true;
            end
            obj.Radius = NurseAgent.RadiusNurse;
            obj.GoalX = x;
            obj.GoalY = y;
            obj.DesiredSpeed = 0;
            obj.Speed = 0;
            obj.State = 'selecting_goal';
        end

        % ============================================================== %
        function setWaypoints(obj, list)
            % list: Nx2 [x y] world coordinates (e.g. nurse station, pharmacy).
            if isempty(list)
                obj.Waypoints = zeros(0,2);
            else
                obj.Waypoints = double(list);
                if size(obj.Waypoints, 2) ~= 2
                    error('NurseAgent:setWaypoints', 'Waypoints must be Nx2.');
                end
            end
        end

        % ============================================================== %
        function step(obj, dt, obstacles, worldW, worldH, nearbyNurses, nearbyRobots)
            % Advance nurse state by dt seconds.
            if nargin < 6 || isempty(nearbyNurses)
                nearbyNurses = zeros(0,3);  % [x y radius]
            end
            if nargin < 7 || isempty(nearbyRobots)
                nearbyRobots = zeros(0,3);  % [x y radius]
            end
            switch obj.State
                case 'selecting_goal'
                    obj.pickNewGoal(obstacles, worldW, worldH);
                case 'dwelling'
                    obj.DwellTime = obj.DwellTime - dt;
                    if obj.DwellTime <= 0
                        obj.State = 'selecting_goal';
                    end
                case {'cruising', 'avoidance'}
                    obj.walkWithLocalSteering(dt, obstacles, worldW, worldH, nearbyNurses, nearbyRobots);
            end
        end
    end

    % ------------------------------------------------------------------ %
    methods (Access = private)

        % ============================================================== %
        function pickNewGoal(obj, obstacles, worldW, worldH)
            m = NurseAgent.WpMargin;
            % Goal category sampling creates realistic nurse cadence.
            u = rand();
            if u < 0.50
                goalType = 'corridor';
            elseif u < 0.85
                goalType = 'station';
            else
                goalType = 'room';
            end

            [gx, gy, ok] = obj.sampleGoalByType(goalType, obstacles, worldW, worldH, m);
            if ~ok
                [gx, gy, ok] = obj.sampleGoalByType('station', obstacles, worldW, worldH, m);
            end
            if ~ok
                [gx, gy, ok] = obj.sampleGoalByType('corridor', obstacles, worldW, worldH, m);
            end
            if ~ok
                obj.State = 'dwelling';
                obj.DwellTime = NurseAgent.MinDwell;
                obj.DesiredSpeed = 0;
                return;
            end

            obj.GoalX = gx;
            obj.GoalY = gy;
            obj.GoalType = goalType;
            baseSpeed = NurseAgent.MinCruiseSpeed + ...
                rand() * (NurseAgent.MaxCruiseSpeed - NurseAgent.MinCruiseSpeed);
            if obj.HasCart
                baseSpeed = 0.9 * baseSpeed;
            end
            obj.DesiredSpeed = baseSpeed;
            obj.State = 'cruising';
        end

        % ============================================================== %
        function walkWithLocalSteering(obj, dt, obstacles, worldW, worldH, nearbyNurses, nearbyRobots)
            toGoal = [obj.GoalX - obj.X, obj.GoalY - obj.Y];
            distGoal = norm(toGoal);
            if distGoal <= NurseAgent.GoalReachTol
                obj.State = 'dwelling';
                obj.DwellTime = NurseAgent.MinDwell + ...
                    rand() * (NurseAgent.MaxDwell - NurseAgent.MinDwell);
                obj.DesiredSpeed = 0;
                return;
            end

            desiredDir = toGoal / max(distGoal, 1e-9);
            obstVec = obj.obstacleRepulsion(obstacles);
            edgeVec = obj.boundaryRepulsion(worldW, worldH);
            neighVec = obj.neighborRepulsion(nearbyNurses, nearbyRobots);

            avoidGain = min(1.0, norm(obstVec + edgeVec + neighVec));
            if avoidGain > 0.25
                obj.State = 'avoidance';
                obj.AvoidTime = max(obj.AvoidTime, 0.3);
            end

            steer = desiredDir + 1.55 * obstVec + 1.20 * neighVec + 1.10 * edgeVec;
            if norm(steer) < 1e-9
                steer = desiredDir;
            end
            steer = steer / max(norm(steer), 1e-9);

            targetTheta = atan2(steer(2), steer(1));
            dTheta = wrapToPi(targetTheta - obj.Theta);
            maxDTheta = NurseAgent.MaxTurnRate * dt;
            dTheta = max(-maxDTheta, min(maxDTheta, dTheta));
            obj.Theta = wrapToPi(obj.Theta + dTheta);

            if strcmp(obj.State, 'avoidance')
                obj.AvoidTime = max(0, obj.AvoidTime - dt);
                speedTarget = obj.DesiredSpeed * NurseAgent.AvoidSpeedScale;
                if obj.AvoidTime <= 0 && avoidGain < 0.12
                    obj.State = 'cruising';
                end
            else
                speedTarget = obj.DesiredSpeed;
            end
            align = max(0.35, cos(wrapToPi(targetTheta - obj.Theta)));
            speedTarget = speedTarget * align;

            maxDv = NurseAgent.MaxAccel * dt;
            dv = max(-maxDv, min(maxDv, speedTarget - obj.Speed));
            obj.Speed = max(0, obj.Speed + dv);

            stepSize = obj.Speed * dt;
            if stepSize < 1e-6
                return;
            end
            trialX = obj.X + stepSize * cos(obj.Theta);
            trialY = obj.Y + stepSize * sin(obj.Theta);
            [nx, ny, moved] = obj.resolveStepWithSliding(trialX, trialY, obstacles, worldW, worldH);
            if ~moved
                obj.State = 'selecting_goal';
                obj.DwellTime = 0;
                obj.Speed = 0;
                return;
            end
            obj.X = nx;
            obj.Y = ny;
        end

        % ============================================================== %
        function tf = insideAnyObstacle(obj, x, y, obstacles)
            tf = false;
            for w = 1:size(obstacles,1)
                if obj.circleAABBOverlap(x, y, obj.Radius, obstacles(w,:))
                    tf = true; return;
                end
            end
        end

        % ============================================================== %
        function tf = circleAABBOverlap(~, cx, cy, cr, aabb)
            dx = abs(cx - aabb(1)); dy = abs(cy - aabb(2));
            hw = aabb(3);           hh = aabb(4);
            if dx > hw + cr || dy > hh + cr, tf = false; return; end
            if dx <= hw || dy <= hh,         tf = true;  return; end
            tf = (dx-hw)^2 + (dy-hh)^2 <= cr^2;
        end

        % ============================================================== %
        function [gx, gy, ok] = sampleGoalByType(obj, goalType, obstacles, worldW, worldH, margin)
            gx = obj.X;
            gy = obj.Y;
            ok = false;
            nWp = size(obj.Waypoints, 1);
            if nWp == 0
                for a = 1:60
                    tx = margin + rand() * (worldW - 2*margin);
                    ty = margin + rand() * (worldH - 2*margin);
                    if ~obj.insideAnyObstacle(tx, ty, obstacles)
                        gx = tx; gy = ty; ok = true; return;
                    end
                end
                return;
            end

            y = obj.Waypoints(:,2);
            corridorMask = abs(y - 31) < 2.4 | abs(y - 20) < 2.4;
            stationMask = abs(y - 12.5) < 1.5;
            roomMask = ~corridorMask;

            switch goalType
                case 'corridor'
                    candidates = find(corridorMask);
                case 'station'
                    candidates = find(stationMask);
                otherwise
                    candidates = find(roomMask);
            end
            if isempty(candidates)
                candidates = 1:nWp;
            end
            for a = 1:100
                idx = candidates(randi(numel(candidates)));
                tx = obj.Waypoints(idx,1) + 0.22 * randn();
                ty = obj.Waypoints(idx,2) + 0.22 * randn();
                tx = max(margin, min(worldW - margin, tx));
                ty = max(margin, min(worldH - margin, ty));
                if ~obj.insideAnyObstacle(tx, ty, obstacles)
                    gx = tx; gy = ty; ok = true; return;
                end
            end
        end

        % ============================================================== %
        function v = obstacleRepulsion(obj, obstacles)
            v = [0, 0];
            rs = NurseAgent.ObstacleSense;
            for i = 1:size(obstacles,1)
                c = obstacles(i,1:2);
                h = obstacles(i,3:4);
                cx = max(c(1)-h(1), min(c(1)+h(1), obj.X));
                cy = max(c(2)-h(2), min(c(2)+h(2), obj.Y));
                d = [obj.X - cx, obj.Y - cy];
                dist = norm(d);
                clearDist = dist - obj.Radius;
                if clearDist < rs
                    dir = d / max(dist, 1e-6);
                    gain = (rs - clearDist) / rs;
                    v = v + 1.2 * gain * dir;
                end
            end
            nv = norm(v);
            if nv > 1
                v = v / nv;
            end
        end

        % ============================================================== %
        function v = boundaryRepulsion(obj, worldW, worldH)
            v = [0, 0];
            rs = NurseAgent.EdgeSense;
            dl = obj.X - obj.Radius;
            dr = worldW - obj.Radius - obj.X;
            db = obj.Y - obj.Radius;
            dt = worldH - obj.Radius - obj.Y;
            if dl < rs, v = v + [ (rs - dl) / rs, 0]; end
            if dr < rs, v = v + [-(rs - dr) / rs, 0]; end
            if db < rs, v = v + [0, (rs - db) / rs]; end
            if dt < rs, v = v + [0, -(rs - dt) / rs]; end
            nv = norm(v);
            if nv > 1
                v = v / nv;
            end
        end

        % ============================================================== %
        function v = neighborRepulsion(~, nearbyNurses, nearbyRobots)
            v = [0, 0];
            rs = NurseAgent.NeighborSense;
            for i = 1:size(nearbyNurses,1)
                d = -nearbyNurses(i,1:2);
                dist = norm(d) - nearbyNurses(i,3);
                if dist < rs
                    dir = d / max(norm(d), 1e-6);
                    v = v + 0.9 * ((rs - dist) / rs) * dir;
                end
            end
            for i = 1:size(nearbyRobots,1)
                d = -nearbyRobots(i,1:2);
                dist = norm(d) - nearbyRobots(i,3);
                if dist < rs
                    dir = d / max(norm(d), 1e-6);
                    v = v + 0.55 * ((rs - dist) / rs) * dir;
                end
            end
            nv = norm(v);
            if nv > 1
                v = v / nv;
            end
        end

        % ============================================================== %
        function [x, y, moved] = resolveStepWithSliding(obj, trialX, trialY, obstacles, worldW, worldH)
            x = max(obj.Radius, min(worldW - obj.Radius, trialX));
            y = max(obj.Radius, min(worldH - obj.Radius, trialY));
            moved = true;
            if ~obj.insideAnyObstacle(x, y, obstacles)
                return;
            end

            motion = [x - obj.X, y - obj.Y];
            if norm(motion) < 1e-9
                moved = false;
                x = obj.X; y = obj.Y;
                return;
            end
            tangents = [ motion(2), -motion(1); -motion(2), motion(1) ];
            for i = 1:2
                t = tangents(i,:) / max(norm(tangents(i,:)), 1e-9);
                cand = [obj.X, obj.Y] + 0.8 * norm(motion) * t;
                cx = max(obj.Radius, min(worldW - obj.Radius, cand(1)));
                cy = max(obj.Radius, min(worldH - obj.Radius, cand(2)));
                if ~obj.insideAnyObstacle(cx, cy, obstacles)
                    x = cx;
                    y = cy;
                    return;
                end
            end

            x = obj.X;
            y = obj.Y;
            moved = false;
        end
    end
end
