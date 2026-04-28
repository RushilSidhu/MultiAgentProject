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
        % When non-empty, waypoints are sampled from this list before random fallbacks.
        Waypoints (:,:) double = zeros(0,2)
    end

    properties (SetAccess = private)
        % Pose  (nurse is modelled as a circle; carts as a larger circle)
        X      (1,1) double
        Y      (1,1) double
        Theta  (1,1) double = 0   % heading [rad]

        % Physical footprint
        Radius  (1,1) double = 0.25   % [m]  plain nurse
        HasCart (1,1) logical = false

        % Navigation state
        WaypointX  (1,1) double
        WaypointY  (1,1) double
        Speed      (1,1) double       % [m/s] current leg speed
        DwellTime  (1,1) double = 0   % [s]  remaining pause time
        State      (1,:) char = 'moving'   % 'moving' | 'dwelling'
    end

    properties (Constant, Access = private)
        % Nurse behaviour parameters
        MinSpeed     = 0.3   % [m/s]
        MaxSpeed     = 1.0
        MinDwell     = 1.0   % [s]
        MaxDwell     = 8.0
        WpMargin     = 2.0   % [m]  keep waypoints away from world edges
        RadiusNurse  = 0.25  % [m]
        RadiusCart   = 0.50  % [m]  nurse+cart
    end

    % ------------------------------------------------------------------ %
    methods
        % ============================================================== %
        function obj = NurseAgent(x, y, hasCart)
            obj.X = x;
            obj.Y = y;
            if nargin >= 3 && hasCart
                obj.HasCart = true;
                obj.Radius  = NurseAgent.RadiusCart;
            else
                obj.Radius  = NurseAgent.RadiusNurse;
            end
            % Bootstrap with a dummy waypoint so the first call to step()
            % immediately samples a real one.
            obj.WaypointX = x;
            obj.WaypointY = y;
            obj.Speed     = 0;
            obj.State     = 'dwelling';
            obj.DwellTime = 0;   % expire immediately → pick waypoint at step 1
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
        function step(obj, dt, obstacles, worldW, worldH)
            % Advance nurse state by dt seconds.
            switch obj.State
                case 'dwelling'
                    obj.DwellTime = obj.DwellTime - dt;
                    if obj.DwellTime <= 0
                        obj.pickNewWaypoint(obstacles, worldW, worldH);
                    end

                case 'moving'
                    obj.walkTowardWaypoint(dt, obstacles, worldW, worldH);
            end
        end
    end

    % ------------------------------------------------------------------ %
    methods (Access = private)

        % ============================================================== %
        function pickNewWaypoint(obj, obstacles, worldW, worldH)
            m = NurseAgent.WpMargin;
            nWp = size(obj.Waypoints, 1);

            % Prefer semantic waypoints from the environment (hospital POIs).
            if nWp > 0
                for attempt = 1:120
                    j = randi(nWp);
                    wx = obj.Waypoints(j, 1);
                    wy = obj.Waypoints(j, 2);
                    if ~obj.insideAnyObstacle(wx, wy, obstacles)
                        obj.WaypointX = wx;
                        obj.WaypointY = wy;
                        obj.Speed     = NurseAgent.MinSpeed + ...
                            rand() * (NurseAgent.MaxSpeed - NurseAgent.MinSpeed);
                        obj.State     = 'moving';
                        return;
                    end
                end
            end

            % Fallback: random free point anywhere in the world.
            for attempt = 1:100
                wx = m + rand() * (worldW - 2*m);
                wy = m + rand() * (worldH - 2*m);
                if ~obj.insideAnyObstacle(wx, wy, obstacles)
                    obj.WaypointX = wx;
                    obj.WaypointY = wy;
                    obj.Speed     = NurseAgent.MinSpeed + ...
                        rand() * (NurseAgent.MaxSpeed - NurseAgent.MinSpeed);
                    obj.State     = 'moving';
                    return;
                end
            end
            % Fallback: stay put and dwell again
            obj.DwellTime = NurseAgent.MinDwell;
            obj.State     = 'dwelling';
        end

        % ============================================================== %
        function walkTowardWaypoint(obj, dt, obstacles, worldW, worldH)
            dx = obj.WaypointX - obj.X;
            dy = obj.WaypointY - obj.Y;
            dist = norm([dx, dy]);

            if dist < 0.15
                % Reached waypoint → dwell
                obj.State     = 'dwelling';
                obj.DwellTime = NurseAgent.MinDwell + ...
                    rand() * (NurseAgent.MaxDwell - NurseAgent.MinDwell);
                return;
            end

            % Unit direction
            ux = dx / dist;
            uy = dy / dist;

            % Proposed new position
            step_size = obj.Speed * dt;
            nx = obj.X + ux * step_size;
            ny = obj.Y + uy * step_size;

            % Clamp to world bounds
            nx = max(obj.Radius, min(worldW - obj.Radius, nx));
            ny = max(obj.Radius, min(worldH - obj.Radius, ny));

            % Wall bounce: if proposed position is inside an obstacle,
            % try reflecting velocity along x, then y, then stop.
            if obj.insideAnyObstacle(nx, ny, obstacles)
                % Try reflecting x
                nx2 = obj.X - ux * step_size;
                ny2 = obj.Y + uy * step_size;
                nx2 = max(obj.Radius, min(worldW-obj.Radius, nx2));
                ny2 = max(obj.Radius, min(worldH-obj.Radius, ny2));
                if ~obj.insideAnyObstacle(nx2, ny2, obstacles)
                    nx = nx2; ny = ny2;
                else
                    % Try reflecting y
                    nx3 = obj.X + ux * step_size;
                    ny3 = obj.Y - uy * step_size;
                    nx3 = max(obj.Radius, min(worldW-obj.Radius, nx3));
                    ny3 = max(obj.Radius, min(worldH-obj.Radius, ny3));
                    if ~obj.insideAnyObstacle(nx3, ny3, obstacles)
                        nx = nx3; ny = ny3;
                    else
                        % Can't move – pick new waypoint next iteration
                        obj.State     = 'dwelling';
                        obj.DwellTime = 0.5;
                        return;
                    end
                end
            end

            obj.X     = nx;
            obj.Y     = ny;
            obj.Theta = atan2(uy, ux);
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
    end
end
