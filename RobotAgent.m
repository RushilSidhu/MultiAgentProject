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

        % Robot status  'idle' | 'busy' | 'error'
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
            obj.Status        = 'idle';
        end

        function setControlMode(obj, mode)
            allowed = {'default', 'conservative_speed', 'aggressive_speed'};
            if ~any(strcmp(mode, allowed))
                error('RobotAgent:InvalidControlMode', ...
                    'Unsupported control mode: %s', mode);
            end
            obj.ControlMode = mode;
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

            if isnan(target_x) || isnan(target_y)
                v = 0; omega = 0;
                return;
            end

            % Bearing to target in world frame
            dx = target_x - obj.X;
            dy = target_y - obj.Y;
            targetAngle = atan2(dy, dx);

            % Heading error (wrapped to [-pi, pi])
            headingErr = wrapToPi(targetAngle - obj.Theta);

            % Mode-specific gains / speed policy.
            nearObstacle = min(lidar_data.ranges) < 1.2;
            switch obj.ControlMode
                case 'conservative_speed'
                    Kp_omega = 2.2;
                    speedScale = 0.65;
                    if nearObstacle
                        speedScale = 0.35;
                    end
                case 'aggressive_speed'
                    Kp_omega = 3.0;
                    speedScale = 1.0;
                    if nearObstacle
                        speedScale = 0.85;
                    end
                otherwise
                    Kp_omega = 2.0;
                    speedScale = 1.0;
                    if nearObstacle
                        speedScale = 0.75;
                    end
            end

            omega = Kp_omega * headingErr;
            omega = max(-obj.MaxAngSpeed, min(obj.MaxAngSpeed, omega));

            % Drive forward only when roughly aligned (reduces arc overshoots)
            alignFactor = max(0, cos(headingErr));
            dist = norm([dx, dy]);
            v = obj.MaxLinSpeed * speedScale * alignFactor * min(1, dist / 2.0);
        end

        % ============================================================== %
        function move(obj, v, omega, dt, obstacles, worldW, worldH)
            % move  Integrate unicycle kinematics; enforce world bounds.
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
