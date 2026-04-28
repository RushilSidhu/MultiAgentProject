classdef HospitalEnv < handle
    % HospitalEnv  2-D top-down hospital simulation environment.
    %
    %   Maintains the world geometry, all moving entities, and the
    %   rendering pipeline.  All entity arrays are stored as handle-object
    %   arrays so state mutations propagate automatically.
    %
    %   Usage
    %   -----
    %     env = HospitalEnv(width, height);
    %     env.addRobot(r);
    %     env.addNurse(n);
    %     env.render();

    % ------------------------------------------------------------------ %
    properties (SetAccess = private)
        Width   (1,1) double = 60      % [m] world x-extent (ward layout)
        Height  (1,1) double = 40      % [m] world y-extent

        % Static obstacle list – each row: [x_centre, y_centre, half_w, half_h]
        % (axis-aligned rectangles).  Concatenation of walls + beds + furniture.
        Obstacles (:,4) double

        % Semantic ward layout (populated in constructor)
        Rooms           struct  % .name .type .xmin .xmax .ymin .ymax .floorColor .labelX .labelY
        POIs            struct  % .id .name .x .y .roomName .taskTarget .nurseStop
        NurseWaypoints  (:,2) double = zeros(0,2)

        % Entity arrays (handle objects – reference semantics)
        Robots  (:,1)               % RobotAgent array
        Nurses  (:,1)               % NurseAgent array

        % Task queue – id, x, y, assignedTo, spawnTime, poiName
        Tasks   struct

        % Indices into POIs array for delivery goals (taskTarget)
        TaskTargetIdx (:,1) double = zeros(0,1)

        % Metrics
        TotalCollisions (1,1) double = 0
        CompletionTimes (:,1) double         % [s] one entry per completed task
        SimTime         (1,1) double = 0     % [s]
        SpeedFactor     (1,1) double = 1.0   % runtime speed multiplier
    end

    properties (Access = private)
        Fig             % figure handle
        Ax              % axes handle
        hObstacles      % patch handles for obstacles
        hRobots         % robot body patch handles
        hRobotHeadings  % robot heading line handles
        hNurseBodies    % nurse body patch handles
        hNurseCarts     % nurse cart patch handles
        hNurseHeadings  % nurse heading line handles
        hTasks          % scatter handle for pending tasks
        hLidar          % line handles for lidar rays
        hTitle          % text handle for metrics overlay
        hLegend         % legend handle
        hSpeedSlider    % slider UI handle
        hSpeedLabel     % text UI handle
        LidarVisible (1,1) logical = true
        RenderingEnabled (1,1) logical = true

        NextTaskID (1,1) double = 1
        AssignmentPolicy (1,:) char = 'nearest_idle'
        NextRoundRobinIdx (1,1) double = 1

        % Static geometry subsets (same row format as Obstacles) for rendering
        WallObstacles      (:,4) double = zeros(0,4)
        BedObstacles       (:,4) double = zeros(0,4)
        FurnitureObstacles (:,4) double = zeros(0,4)

        hRoomFloors   % gobjects vector – translucent room floor patches
        hBedPatches   % gobjects vector
        hFurnPatches  % gobjects vector
        hPoiMarkers   % scatter handle for task POI sites
        hRoomLabels   % text handles for room names
    end

    % ------------------------------------------------------------------ %
    methods
        % ============================================================== %
        function obj = HospitalEnv(width, height, assignmentPolicy, enableRendering)
            if nargin >= 1, obj.Width  = width;  end
            if nargin >= 2, obj.Height = height; end
            if nargin >= 3 && ~isempty(assignmentPolicy)
                obj.AssignmentPolicy = assignmentPolicy;
            end
            if nargin >= 4
                obj.RenderingEnabled = logical(enableRendering);
            end

            obj.Robots = RobotAgent.empty(0,1);
            obj.Nurses = NurseAgent.empty(0,1);
            obj.Tasks  = struct('id',{},'x',{},'y',{},'assignedTo',{},...
                                'spawnTime',{},'poiName',{});

            obj.buildDefaultObstacles();
            obj.buildRooms();
            obj.buildPOIs();
            if obj.RenderingEnabled
                obj.initFigure();
            end
        end

        % ============================================================== %
        function addRobot(obj, robot)
            obj.Robots(end+1,1) = robot;
        end

        function addNurse(obj, nurse)
            obj.Nurses(end+1,1) = nurse;
            if ~isempty(obj.NurseWaypoints)
                nurse.setWaypoints(obj.NurseWaypoints);
            end
        end

        % ============================================================== %
        function spawnTask(obj)
            % Place a task at a semantic POI (bed, ICU, exam, pharmacy, supply).
            [pos, poiName] = obj.sampleTaskPOI();
            t.id         = obj.NextTaskID;
            t.x          = pos(1);
            t.y          = pos(2);
            t.assignedTo = -1;       % -1 = unassigned
            t.spawnTime  = obj.SimTime;
            t.poiName    = poiName;
            obj.NextTaskID = obj.NextTaskID + 1;

            chosen = obj.selectIdleRobotForTask(t.x, t.y);
            if chosen > 0
                obj.Robots(chosen).assignTarget(t.x, t.y, t.id);
                t.assignedTo = chosen;
                fprintf('[%.2fs] Task %d spawned at %s (%.1f,%.1f) → Robot %d\n', ...
                    obj.SimTime, t.id, t.poiName, t.x, t.y, chosen);
            else
                fprintf('[%.2fs] Task %d spawned at %s (%.1f,%.1f) → QUEUED (no idle robot)\n', ...
                    obj.SimTime, t.id, t.poiName, t.x, t.y);
            end

            obj.Tasks(end+1) = t;
        end

        % ============================================================== %
        function step(obj, dt)
            % Advance simulation by dt seconds.
            obj.SimTime = obj.SimTime + dt;

            % 1. Update nurses (dynamic obstacles)
            for k = 1:numel(obj.Nurses)
                obj.Nurses(k).step(dt, obj.Obstacles, obj.Width, obj.Height);
            end

            % 2. Build combined obstacle list for Lidar (walls + nurses + other robots)
            for i = 1:numel(obj.Robots)
                otherRobots = obj.Robots([1:i-1, i+1:end]);
                lidarData   = obj.Robots(i).senseEnvironment(...
                    obj.Obstacles, obj.Nurses, otherRobots);

                [v, omega] = obj.Robots(i).calculateVelocity(...
                    obj.Robots(i).TargetX, obj.Robots(i).TargetY, lidarData);

                obj.Robots(i).move(v, omega, dt, obj.Obstacles, obj.Width, obj.Height);
            end

            % 3. Collision detection
            obj.detectCollisions();

            % 4. Task completion check
            obj.checkTaskCompletion();

            % 5. Reassign queued tasks if robots became idle
            obj.reassignQueuedTasks();
        end

        % ============================================================== %
        function render(obj)
            % High-performance render using pre-allocated graphics objects.
            if ~obj.RenderingEnabled || isempty(obj.Ax) || ~isgraphics(obj.Ax)
                return;
            end

            % -- Lidar rays (reuse handles; update X/Y only) ----------
            for i = 1:numel(obj.Robots)
                r = obj.Robots(i);
                if isempty(r.LidarAngles)
                    continue;
                end
                [xray, yray] = obj.packLidarSegments(r);
                if i <= numel(obj.hLidar) && isvalid(obj.hLidar(i))
                    set(obj.hLidar(i), 'XData', xray, 'YData', yray, ...
                        'Visible', obj.boolToOnOff(obj.LidarVisible));
                else
                    obj.hLidar(i) = line(obj.Ax, xray, yray, ...
                        'Color', [0.35 0.85 0.35], ...
                        'LineWidth', 0.35, ...
                        'Visible', obj.boolToOnOff(obj.LidarVisible));
                end
            end

            % -- Robots ------------------------------------------------
            for i = 1:numel(obj.Robots)
                r  = obj.Robots(i);
                th = linspace(0, 2*pi, 24);
                px = r.X + r.Radius*cos(th);
                py = r.Y + r.Radius*sin(th);
                if i <= numel(obj.hRobots) && isvalid(obj.hRobots(i))
                    set(obj.hRobots(i), 'XData', px, 'YData', py);
                else
                    col = obj.robotColor(r.Status);
                    obj.hRobots(i) = fill(obj.Ax, px, py, col, ...
                        'EdgeColor',[0.05 0.05 0.05],'LineWidth',1.4);
                end
                % Update colour based on status
                set(obj.hRobots(i), 'FaceColor', obj.robotColor(r.Status));

                % Heading indicator
                arrowLen = r.Radius * 1.7;
                hx = [r.X, r.X + arrowLen*cos(r.Theta)];
                hy = [r.Y, r.Y + arrowLen*sin(r.Theta)];
                if i <= numel(obj.hRobotHeadings) && isgraphics(obj.hRobotHeadings(i))
                    set(obj.hRobotHeadings(i), 'XData', hx, 'YData', hy, 'Visible','on');
                else
                    obj.hRobotHeadings(i) = line(obj.Ax, hx, hy, ...
                        'Color',[0.0 0.0 0.0], 'LineWidth',1.6);
                end
            end

            % -- Nurses ------------------------------------------------
            for k = 1:numel(obj.Nurses)
                n  = obj.Nurses(k);
                [bodyX, bodyY, cartX, cartY] = obj.nurseGlyph(n);

                if n.HasCart
                    if k <= numel(obj.hNurseCarts) && isgraphics(obj.hNurseCarts(k))
                        set(obj.hNurseCarts(k), 'XData', cartX, 'YData', cartY, 'Visible','on');
                    else
                        obj.hNurseCarts(k) = fill(obj.Ax, cartX, cartY, [0.30 0.66 0.98], ...
                            'EdgeColor',[0.85 0.94 1.0], 'LineWidth',1.0);
                    end
                else
                    if k <= numel(obj.hNurseCarts) && isgraphics(obj.hNurseCarts(k))
                        set(obj.hNurseCarts(k), 'Visible','off');
                    end
                end

                if k <= numel(obj.hNurseBodies) && isgraphics(obj.hNurseBodies(k))
                    set(obj.hNurseBodies(k), 'XData', bodyX, 'YData', bodyY);
                else
                    obj.hNurseBodies(k) = fill(obj.Ax, bodyX, bodyY, [0.16 0.52 0.92], ...
                        'EdgeColor',[0.94 0.98 1.0], 'LineWidth',1.2);
                end

                if n.HasCart
                    set(obj.hNurseBodies(k), 'FaceColor', [0.12 0.44 0.84], ...
                        'EdgeColor', [0.94 0.98 1.0]);
                else
                    set(obj.hNurseBodies(k), 'FaceColor', [0.22 0.78 0.92], ...
                        'EdgeColor', [0.92 1.0 1.0]);
                end

                headingLen = n.Radius * 1.4;
                hx = [n.X, n.X + headingLen*cos(n.Theta)];
                hy = [n.Y, n.Y + headingLen*sin(n.Theta)];
                if k <= numel(obj.hNurseHeadings) && isgraphics(obj.hNurseHeadings(k))
                    set(obj.hNurseHeadings(k), 'XData', hx, 'YData', hy, 'Visible','on');
                else
                    obj.hNurseHeadings(k) = line(obj.Ax, hx, hy, ...
                        'Color',[0.96 0.98 1.0], 'LineWidth',1.2);
                end
            end

            % -- Active tasks (scatter) --------------------------------
            activeMask = arrayfun(@(t) t.assignedTo == -1, obj.Tasks);
            activeTasks = obj.Tasks(activeMask);
            if ~isempty(activeTasks)
                set(obj.hTasks, ...
                    'XData', [activeTasks.x], ...
                    'YData', [activeTasks.y], ...
                    'Visible','on');
            else
                set(obj.hTasks, 'Visible','off');
            end

            % -- Metrics overlay --------------------------------------
            avgTime = 0;
            if ~isempty(obj.CompletionTimes)
                avgTime = mean(obj.CompletionTimes);
            end
            nActive = numel(obj.Tasks) - sum([obj.Tasks.assignedTo] >= 0 & ...
                arrayfun(@(t)~obj.isTaskComplete(t), obj.Tasks));

            set(obj.hTitle, 'String', ...
                sprintf('T=%.1fs | Collisions=%d | AvgTaskTime=%.1fs | ActiveTasks=%d', ...
                obj.SimTime, obj.TotalCollisions, avgTime, numel(obj.Tasks)));

            drawnow limitrate
        end

        % ============================================================== %
        function tf = isFigureOpen(obj)
            % Public query used by main loop to detect user-closed figure.
            tf = ~isempty(obj.Fig) && isvalid(obj.Fig) && ishandle(obj.Fig);
        end

        % ============================================================== %
        function addSpeedControl(obj, initialSpeed, minSpeed, maxSpeed)
            if ~obj.RenderingEnabled
                return;
            end
            if nargin < 2 || isempty(initialSpeed), initialSpeed = 1.0; end
            if nargin < 3 || isempty(minSpeed),     minSpeed = 0.25;    end
            if nargin < 4 || isempty(maxSpeed),     maxSpeed = 8.0;     end

            initialSpeed = min(max(initialSpeed, minSpeed), maxSpeed);
            obj.SpeedFactor = initialSpeed;

            obj.hSpeedSlider = uicontrol(obj.Fig, ...
                'Style', 'slider', ...
                'Units', 'normalized', ...
                'Position', [0.72 0.95 0.22 0.03], ...
                'Min', minSpeed, ...
                'Max', maxSpeed, ...
                'Value', initialSpeed, ...
                'Callback', @(src,~)obj.onSpeedSliderChanged(src));

            obj.hSpeedLabel = uicontrol(obj.Fig, ...
                'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.72 0.91 0.22 0.03], ...
                'String', sprintf('Speed: %.2fx', initialSpeed), ...
                'BackgroundColor', [0.07 0.07 0.12], ...
                'ForegroundColor', 'w', ...
                'HorizontalAlignment', 'left');
        end

        % ============================================================== %
        function setLidarVisible(obj, tf)
            obj.LidarVisible = logical(tf);
            if isempty(obj.hLidar)
                return;
            end
            for i = 1:numel(obj.hLidar)
                if isvalid(obj.hLidar(i))
                    set(obj.hLidar(i), 'Visible', obj.boolToOnOff(obj.LidarVisible));
                end
            end
        end

        % ============================================================== %
        function printMetrics(obj)
            fprintf('\n===== SIMULATION METRICS (T=%.2fs) =====\n', obj.SimTime);
            fprintf('  Total Collisions    : %d\n', obj.TotalCollisions);
            fprintf('  Tasks Completed     : %d\n', numel(obj.CompletionTimes));
            if ~isempty(obj.CompletionTimes)
                fprintf('  Avg Completion Time : %.2f s\n', mean(obj.CompletionTimes));
                fprintf('  Min / Max Time      : %.2f / %.2f s\n', ...
                    min(obj.CompletionTimes), max(obj.CompletionTimes));
            end
            fprintf('  Tasks Still Active  : %d\n', numel(obj.Tasks));
            fprintf('=========================================\n\n');
        end
    end

    % ------------------------------------------------------------------ %
    methods (Access = private)

        % ============================================================== %
        function buildDefaultObstacles(obj)
            % 60 x 40 m double-loaded ward: patient rooms (north), main corridor,
            % clinical block (south), lobby arch (east).  Rows: [cx,cy,hw,hh].
            W = obj.Width; H = obj.Height;
            if abs(W - 60) > 1e-6 || abs(H - 40) > 1e-6
                error('HospitalEnv:buildDefaultObstacles', ...
                    'Ward geometry is defined for Width=60 m, Height=40 m (got %.4g x %.4g).', ...
                    W, H);
            end

            tB = 0.25;   % perimeter half-thickness
            tt = 0.15;   % interior wall half-thickness
            innerL = tB;
            innerR = W - tB;
            innerB = tB;
            innerT = H - tB;

            borders = [
                W/2,     tB/2,   W/2,     tB/2;
                W/2, H - tB/2,   W/2,     tB/2;
                tB/2,   H/2,     tB/2,    H/2;
                W - tB/2, H/2,   tB/2,    H/2;
            ];

            % Patient wing vertical dividers (x centres), y = 24 .. innerT
            xv = (9.5:9:54.5)';
            cyP = (24 + innerT) / 2;
            hhP = (innerT - 24) / 2;
            vertPatient = [xv, repmat(cyP, 6, 1), repmat(tt, 6, 1), repmat(hhP, 6, 1)];

            % Patient–corridor wall at y = 24 with door gaps (1.5 m each)
            doorsTop = [
                4.25,  5.75;
                13.25, 14.75;
                22.25, 23.75;
                31.25, 32.75;
                40.25, 41.75;
                49.25, 50.75;
                56.5,  58.0;
            ];
            obj.assertDoorWidths(doorsTop, 1.2);
            horizPatient = obj.horizontalWallSegments(24, innerL, innerR, doorsTop, tt);

            % Clinical block vertical dividers, y = innerB .. 18
            xc = [14; 22; 30; 41; 52];
            cyC = (innerB + 18) / 2;
            hhC = (18 - innerB) / 2;
            vertClinical = [xc, repmat(cyC, 5, 1), repmat(tt, 5, 1), repmat(hhC, 5, 1)];

            % Clinical–corridor wall at y = 18; lobby arch open for x >= 52
            doorsClinical = [
                5.25,  6.75;
                17.25, 18.75;
                25.25, 26.75;
                34.75, 36.25;
                45.75, 47.25;
            ];
            obj.assertDoorWidths(doorsClinical, 1.2);
            xWallEnd = 52; % open arch / lobby connection for x > 52
            horizClinical = obj.horizontalWallSegments(18, innerL, xWallEnd, doorsClinical, tt);

            obj.WallObstacles = [borders; vertPatient; horizPatient; vertClinical; horizClinical];

            % Beds (distinct render colour)
            bedHW = 1.2; bedHH = 0.5;
            patCx = (5:9:50)';
            patientBeds = [patCx, repmat(38.5, 6, 1), repmat(bedHW, 6, 1), repmat(bedHH, 6, 1)];
            examBed = [57, 38.5, 1.0, 0.6];
            icuCx = [34; 38; 45; 49];
            icuBeds = [icuCx, repmat(2.5, 4, 1), repmat(0.6, 4, 1), repmat(1.2, 4, 1)];
            obj.BedObstacles = [patientBeds; examBed; icuBeds];

            % Furniture / benches (collision + render)
            counter   = [7.25, 15.5, 4.0, 0.5];
            pharmShel = [18,   14,   2.0, 0.5];
            suppShel  = [26,   14,   2.0, 0.5];
            bench1    = [55.75, 10,   2.0, 0.35];
            bench2    = [55.75, 6,    2.0, 0.35];
            obj.FurnitureObstacles = [counter; pharmShel; suppShel; bench1; bench2];

            obj.Obstacles = [obj.WallObstacles; obj.BedObstacles; obj.FurnitureObstacles];
        end

        % ============================================================== %
        function buildRooms(obj)
            cPatient  = [0.10 0.18 0.30];
            cICU      = [0.30 0.12 0.12];
            cNurse    = [0.10 0.28 0.18];
            cPharm    = [0.30 0.24 0.10];
            cSupply   = [0.18 0.20 0.28];
            cLobby    = [0.14 0.14 0.18];
            cCorridor = [0.11 0.12 0.16];

            R = struct('name',{},'type',{},'xmin',{},'xmax',{},'ymin',{},'ymax',{}, ...
                'floorColor',{},'labelX',{},'labelY',{});
            R(end+1) = struct('name','P101','type','patient', ...
                'xmin',0.25,'xmax',9.35,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',5,'labelY',36.5);
            R(end+1) = struct('name','P102','type','patient', ...
                'xmin',9.65,'xmax',18.35,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',14,'labelY',36.5);
            R(end+1) = struct('name','P103','type','patient', ...
                'xmin',18.65,'xmax',27.35,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',23,'labelY',36.5);
            R(end+1) = struct('name','P104','type','patient', ...
                'xmin',27.65,'xmax',36.35,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',32,'labelY',36.5);
            R(end+1) = struct('name','P105','type','patient', ...
                'xmin',36.65,'xmax',45.35,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',41,'labelY',36.5);
            R(end+1) = struct('name','P106','type','patient', ...
                'xmin',45.65,'xmax',54.35,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',50,'labelY',36.5);
            R(end+1) = struct('name','EXAM','type','patient', ...
                'xmin',54.65,'xmax',59.75,'ymin',24,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',57,'labelY',36.5);
            R(end+1) = struct('name','Corridor','type','corridor', ...
                'xmin',0.25,'xmax',59.75,'ymin',18,'ymax',24, ...
                'floorColor',cCorridor,'labelX',30,'labelY',21);
            R(end+1) = struct('name','Nurse Station','type','nurse', ...
                'xmin',0.25,'xmax',13.85,'ymin',0.25,'ymax',17.85, ...
                'floorColor',cNurse,'labelX',7,'labelY',10);
            R(end+1) = struct('name','Pharmacy','type','pharmacy', ...
                'xmin',14.15,'xmax',21.85,'ymin',0.25,'ymax',17.85, ...
                'floorColor',cPharm,'labelX',18,'labelY',10);
            R(end+1) = struct('name','Supply','type','supply', ...
                'xmin',22.15,'xmax',29.85,'ymin',0.25,'ymax',17.85, ...
                'floorColor',cSupply,'labelX',26,'labelY',10);
            R(end+1) = struct('name','ICU-A','type','icu', ...
                'xmin',30.15,'xmax',40.85,'ymin',0.25,'ymax',17.85, ...
                'floorColor',cICU,'labelX',35.5,'labelY',10);
            R(end+1) = struct('name','ICU-B','type','icu', ...
                'xmin',41.15,'xmax',51.85,'ymin',0.25,'ymax',17.85, ...
                'floorColor',cICU,'labelX',46.5,'labelY',10);
            R(end+1) = struct('name','Lobby','type','lobby', ...
                'xmin',52.15,'xmax',59.75,'ymin',0.25,'ymax',23.8, ...
                'floorColor',cLobby,'labelX',56,'labelY',12);
            obj.Rooms = R;
        end

        % ============================================================== %
        function buildPOIs(obj)
            % Delivery targets + nurse tour stops (world frame, metres).
            names = { ...
                'bedside_P101'; 'bedside_P102'; 'bedside_P103'; 'bedside_P104'; ...
                'bedside_P105'; 'bedside_P106'; 'exam_table'; ...
                'bedside_ICUA_1'; 'bedside_ICUA_2'; 'bedside_ICUB_1'; 'bedside_ICUB_2'; ...
                'pharmacy_pickup'; 'supply_pickup'; 'nurse_desk'; 'corridor_mid' };
            xs = [5; 14; 23; 32; 41; 50; 57; 34; 38; 45; 49; 18; 26; 7.25; 30];
            ys = [37; 37; 37; 37; 37; 37; 36.5; 4; 4; 4; 4; 12; 12; 13.5; 21];
            rms = { 'P101'; 'P102'; 'P103'; 'P104'; 'P105'; 'P106'; 'EXAM'; ...
                'ICU-A'; 'ICU-A'; 'ICU-B'; 'ICU-B'; 'Pharmacy'; 'Supply'; ...
                'Nurse Station'; 'Corridor' };
            tt  = [true(7,1); true(4,1); true(2,1); false; false];
            ns  = true(15,1);

            p = repmat(struct('id',0,'name','','x',0,'y',0,'roomName','', ...
                'taskTarget',false,'nurseStop',false), numel(names), 1);
            for k = 1:numel(names)
                p(k) = struct('id', k, 'name', names{k}, 'x', xs(k), 'y', ys(k), ...
                    'roomName', rms{k}, 'taskTarget', logical(tt(k)), ...
                    'nurseStop', logical(ns(k)));
            end
            obj.POIs = p;
            obj.TaskTargetIdx = find([obj.POIs.taskTarget])';
            nwMask = [obj.POIs.nurseStop];
            obj.NurseWaypoints = [[obj.POIs(nwMask).x]', [obj.POIs(nwMask).y]'];
        end

        % ============================================================== %
        function [pos, poiName] = sampleTaskPOI(obj)
            if isempty(obj.TaskTargetIdx)
                pos = obj.randomFreePosition();
                poiName = 'random';
                return;
            end
            j = obj.TaskTargetIdx(randi(numel(obj.TaskTargetIdx)));
            q = obj.POIs(j);
            pos = [q.x, q.y];
            poiName = q.name;
        end

        % ============================================================== %
        function initFigure(obj)
            obj.Fig = figure('Name','Hospital Simulation','Color','k',...
                'Units','normalized','Position',[0.05 0.05 0.6 0.85]);
            obj.Ax  = axes(obj.Fig, 'Color',[0.07 0.07 0.12],...
                'XColor','w','YColor','w','FontSize',8,...
                'DataAspectRatio',[1 1 1]);
            hold(obj.Ax,'on');
            axis(obj.Ax, [0 obj.Width 0 obj.Height]);
            grid(obj.Ax,'on');
            obj.Ax.GridColor = [0.2 0.2 0.3];

            % --- Room floor tint (below walls) -------------------------
            nR = numel(obj.Rooms);
            obj.hRoomFloors = gobjects(nR, 1);
            for ri = 1:nR
                Rm = obj.Rooms(ri);
                if strcmp(Rm.type, 'corridor')
                    fa = 0.10; % subtle corridor tint
                else
                    fa = 0.28;
                end
                col = Rm.floorColor;
                obj.hRoomFloors(ri) = patch(obj.Ax, ...
                    [Rm.xmin Rm.xmax Rm.xmax Rm.xmin Rm.xmin], ...
                    [Rm.ymin Rm.ymin Rm.ymax Rm.ymax Rm.ymin], ...
                    col, 'FaceAlpha', fa, 'EdgeColor', 'none');
            end

            % --- Structural walls --------------------------------------
            for i = 1:size(obj.WallObstacles,1)
                cx = obj.WallObstacles(i,1); cy = obj.WallObstacles(i,2);
                hw = obj.WallObstacles(i,3); hh = obj.WallObstacles(i,4);
                patch(obj.Ax, ...
                    cx+[-hw hw hw -hw -hw], ...
                    cy+[-hh -hh hh hh -hh], ...
                    [0.52 0.52 0.58], 'EdgeColor', [0.78 0.78 0.86], 'LineWidth', 0.9);
            end

            % --- Beds --------------------------------------------------
            nb = size(obj.BedObstacles,1);
            obj.hBedPatches = gobjects(nb, 1);
            for i = 1:nb
                cx = obj.BedObstacles(i,1); cy = obj.BedObstacles(i,2);
                hw = obj.BedObstacles(i,3); hh = obj.BedObstacles(i,4);
                obj.hBedPatches(i) = patch(obj.Ax, ...
                    cx+[-hw hw hw -hw -hw], ...
                    cy+[-hh -hh hh hh -hh], ...
                    [0.92 0.94 0.98], 'EdgeColor', [0.75 0.80 0.90], 'LineWidth', 0.6);
            end

            % --- Furniture ---------------------------------------------
            nf = size(obj.FurnitureObstacles,1);
            obj.hFurnPatches = gobjects(nf, 1);
            for i = 1:nf
                cx = obj.FurnitureObstacles(i,1); cy = obj.FurnitureObstacles(i,2);
                hw = obj.FurnitureObstacles(i,3); hh = obj.FurnitureObstacles(i,4);
                obj.hFurnPatches(i) = patch(obj.Ax, ...
                    cx+[-hw hw hw -hw -hw], ...
                    cy+[-hh -hh hh hh -hh], ...
                    [0.42 0.44 0.50], 'EdgeColor', [0.65 0.68 0.75], 'LineWidth', 0.7);
            end

            % --- Task POI markers (dim circles at delivery sites) -----
            ttMask = arrayfun(@(q) q.taskTarget, obj.POIs);
            xp = [obj.POIs(ttMask).x];
            yp = [obj.POIs(ttMask).y];
            obj.hPoiMarkers = scatter(obj.Ax, xp, yp, 36, ...
                'Marker','o', 'MarkerEdgeColor', [0.45 0.62 0.82], ...
                'MarkerFaceColor', 'none', 'LineWidth', 0.85);

            % --- Room labels -------------------------------------------
            obj.hRoomLabels = gobjects(nR, 1);
            for ri = 1:nR
                Rm = obj.Rooms(ri);
                obj.hRoomLabels(ri) = text(obj.Ax, Rm.labelX, Rm.labelY, Rm.name, ...
                    'Color', [0.82 0.90 1.0], 'FontSize', 8, ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                    'FontWeight', 'bold');
            end

            % Placeholder graphics containers
            obj.hRobots = gobjects(0);
            obj.hRobotHeadings = gobjects(0);
            obj.hNurseBodies = gobjects(0);
            obj.hNurseCarts = gobjects(0);
            obj.hNurseHeadings = gobjects(0);
            obj.hLidar  = gobjects(0);

            % Task markers
            obj.hTasks = scatter(obj.Ax, NaN, NaN, 140, 'Marker','pentagram', ...
                'MarkerEdgeColor',[1.0 0.25 0.28], ...
                'MarkerFaceColor',[1.0 0.78 0.25], ...
                'LineWidth', 1.3, 'Visible','off');

            % Legend symbols (dummy handles for clear type mapping)
            hLegRobot = fill(obj.Ax, NaN, NaN, [0.95 0.85 0.25], ...
                'EdgeColor',[0.05 0.05 0.05], 'LineWidth',1.4);
            hLegNurse = fill(obj.Ax, NaN, NaN, [0.22 0.78 0.92], ...
                'EdgeColor',[0.92 1.0 1.0], 'LineWidth',1.2);
            hLegCart  = fill(obj.Ax, NaN, NaN, [0.12 0.44 0.84], ...
                'EdgeColor',[0.94 0.98 1.0], 'LineWidth',1.2);
            hLegTask  = scatter(obj.Ax, NaN, NaN, 120, 'Marker','pentagram', ...
                'MarkerEdgeColor',[1.0 0.25 0.28], ...
                'MarkerFaceColor',[1.0 0.78 0.25], ...
                'LineWidth',1.3);
            obj.hLegend = legend(obj.Ax, [hLegRobot hLegNurse hLegCart hLegTask], ...
                {'Robot', 'Nurse', 'Nurse + cart', 'Task'}, ...
                'TextColor','w', 'Location','northeastoutside', ...
                'Box','off', 'FontSize',8);

            % Title / metrics text
            obj.hTitle = title(obj.Ax, 'Initializing...', ...
                'Color','w','FontSize',9,'FontName','Monospaced');

            xlabel(obj.Ax,'x [m]'); ylabel(obj.Ax,'y [m]');
        end

        % ============================================================== %
        function detectCollisions(obj)
            % Entity-entity collisions
            nRobots = numel(obj.Robots);
            nNurses = numel(obj.Nurses);

            % Robot-robot
            for i = 1:nRobots-1
                for j = i+1:nRobots
                    ri = obj.Robots(i);
                    rj = obj.Robots(j);
                    dist = norm([ri.X-rj.X, ri.Y-rj.Y]);
                    if dist < (ri.Radius + rj.Radius)
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] Entity %s vs %s  dist=%.3f\n',...
                            obj.SimTime, class(ri), class(rj), dist);
                    end
                end
            end

            % Nurse-nurse
            for i = 1:nNurses-1
                for j = i+1:nNurses
                    ni = obj.Nurses(i);
                    nj = obj.Nurses(j);
                    dist = norm([ni.X-nj.X, ni.Y-nj.Y]);
                    if dist < (ni.Radius + nj.Radius)
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] Entity %s vs %s  dist=%.3f\n',...
                            obj.SimTime, class(ni), class(nj), dist);
                    end
                end
            end

            % Robot-nurse
            for i = 1:nRobots
                for j = 1:nNurses
                    r = obj.Robots(i);
                    n = obj.Nurses(j);
                    dist = norm([r.X-n.X, r.Y-n.Y]);
                    if dist < (r.Radius + n.Radius)
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] Entity %s vs %s  dist=%.3f\n',...
                            obj.SimTime, class(r), class(n), dist);
                    end
                end
            end

            % Entity-wall (AABB) collisions
            for i = 1:nRobots
                e = obj.Robots(i);
                for w = 1:size(obj.Obstacles,1)
                    if obj.circleAABBOverlap(e.X, e.Y, e.Radius, obj.Obstacles(w,:))
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] %s #%d hit wall #%d\n',...
                            obj.SimTime, class(e), i, w);
                    end
                end
            end

            for i = 1:nNurses
                e = obj.Nurses(i);
                for w = 1:size(obj.Obstacles,1)
                    if obj.circleAABBOverlap(e.X, e.Y, e.Radius, obj.Obstacles(w,:))
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] %s #%d hit wall #%d\n',...
                            obj.SimTime, class(e), i, w);
                    end
                end
            end
        end

        % ============================================================== %
        function checkTaskCompletion(obj)
            acceptRadius = 1.0; % [m]
            for i = 1:numel(obj.Robots)
                r = obj.Robots(i);
                if strcmp(r.Status,'busy') && ~isnan(r.TargetX)
                    d = norm([r.X - r.TargetX, r.Y - r.TargetY]);
                    if d < acceptRadius
                        taskId = r.CurrentTaskID;
                        elapsed = obj.SimTime - obj.getTaskSpawnTime(taskId);
                        obj.CompletionTimes(end+1) = elapsed;
                        fprintf('[COMPLETE %.2fs] Robot %d finished Task %d in %.2fs\n',...
                            obj.SimTime, i, taskId, elapsed);
                        r.clearTarget();
                        obj.removeTask(taskId);
                    end
                end
            end
        end

        % ============================================================== %
        function reassignQueuedTasks(obj)
            % Find tasks with assignedTo == -1 and try to assign
            for ti = 1:numel(obj.Tasks)
                if obj.Tasks(ti).assignedTo == -1
                    chosen = obj.selectIdleRobotForTask(obj.Tasks(ti).x, obj.Tasks(ti).y);
                    if chosen > 0
                        obj.Robots(chosen).assignTarget(...
                            obj.Tasks(ti).x, obj.Tasks(ti).y, obj.Tasks(ti).id);
                        obj.Tasks(ti).assignedTo = chosen;
                        fprintf('[ASSIGN %.2fs] Task %d reassigned → Robot %d\n',...
                            obj.SimTime, obj.Tasks(ti).id, chosen);
                    end
                end
            end
        end

        % ============================================================== %
        function t = getTaskSpawnTime(obj, taskID)
            t = 0;
            for k = 1:numel(obj.Tasks)
                if obj.Tasks(k).id == taskID
                    t = obj.Tasks(k).spawnTime;
                    return;
                end
            end
        end

        % ============================================================== %
        function removeTask(obj, taskID)
            keep = arrayfun(@(t) t.id ~= taskID, obj.Tasks);
            obj.Tasks = obj.Tasks(keep);
        end

        % ============================================================== %
        function tf = isTaskComplete(~, task) %#ok<INUSL>
            tf = false; % tasks are removed on completion; kept for safety check
        end

        % ============================================================== %
        function pos = randomFreePosition(obj)
            margin = 2.0;
            for attempt = 1:200
                x = margin + rand() * (obj.Width  - 2*margin);
                y = margin + rand() * (obj.Height - 2*margin);
                if ~obj.positionInObstacle(x, y, 0.5)
                    pos = [x, y];
                    return;
                end
            end
            pos = [obj.Width/2, obj.Height/2]; % fallback
        end

        % ============================================================== %
        function tf = positionInObstacle(obj, x, y, r)
            tf = false;
            for w = 1:size(obj.Obstacles,1)
                if obj.circleAABBOverlap(x, y, r, obj.Obstacles(w,:))
                    tf = true; return;
                end
            end
        end

        % ============================================================== %
        function tf = circleAABBOverlap(~, cx, cy, cr, aabb)
            % aabb = [box_cx, box_cy, half_w, half_h]
            dx = abs(cx - aabb(1));
            dy = abs(cy - aabb(2));
            hw = aabb(3); hh = aabb(4);
            if dx > hw + cr || dy > hh + cr
                tf = false; return;
            end
            if dx <= hw || dy <= hh
                tf = true; return;
            end
            cornerDist2 = (dx - hw)^2 + (dy - hh)^2;
            tf = cornerDist2 <= cr^2;
        end

        % ============================================================== %
        function c = robotColor(~, status)
            switch status
                case 'idle',  c = [0.95 0.85 0.25];
                case 'busy',  c = [0.28 0.92 0.45];
                case 'error', c = [0.96 0.24 0.24];
                otherwise,    c = [0.72 0.72 0.72];
            end
        end

        % ============================================================== %
        function [bodyX, bodyY, cartX, cartY] = nurseGlyph(~, nurse)
            th = linspace(0, 2*pi, 28);
            bodyR = nurse.Radius * (0.62 + 0.18 * nurse.HasCart);
            bodyX = nurse.X + bodyR*cos(th);
            bodyY = nurse.Y + bodyR*sin(th);

            if nurse.HasCart
                u = [cos(nurse.Theta), sin(nurse.Theta)];
                p = [-u(2), u(1)];

                cartLen = bodyR * 1.9;
                cartWid = bodyR * 1.15;
                cartCenter = [nurse.X, nurse.Y] + u * (bodyR + 0.45*cartLen);

                c1 = cartCenter + 0.5*cartLen*u + 0.5*cartWid*p;
                c2 = cartCenter + 0.5*cartLen*u - 0.5*cartWid*p;
                c3 = cartCenter - 0.5*cartLen*u - 0.5*cartWid*p;
                c4 = cartCenter - 0.5*cartLen*u + 0.5*cartWid*p;
                cartX = [c1(1) c2(1) c3(1) c4(1) c1(1)];
                cartY = [c1(2) c2(2) c3(2) c4(2) c1(2)];
            else
                cartX = NaN;
                cartY = NaN;
            end
        end

        % ============================================================== %
        function [xray, yray] = packLidarSegments(~, robot)
            n = numel(robot.LidarRanges);
            xray = nan(1, 3*n);
            yray = nan(1, 3*n);
            if n == 0
                return;
            end

            endX = robot.X + robot.LidarRanges .* cos(robot.LidarAngles + robot.Theta);
            endY = robot.Y + robot.LidarRanges .* sin(robot.LidarAngles + robot.Theta);

            idx0 = 1:3:(3*n);
            idx1 = 2:3:(3*n);
            xray(idx0) = robot.X;
            xray(idx1) = endX;
            yray(idx0) = robot.Y;
            yray(idx1) = endY;
        end

        % ============================================================== %
        function s = boolToOnOff(~, tf)
            if tf
                s = 'on';
            else
                s = 'off';
            end
        end

        % ============================================================== %
        function onSpeedSliderChanged(obj, src)
            obj.SpeedFactor = src.Value;
            if ~isempty(obj.hSpeedLabel) && isvalid(obj.hSpeedLabel)
                obj.hSpeedLabel.String = sprintf('Speed: %.2fx', obj.SpeedFactor);
            end
        end

        % ============================================================== %
        function rows = horizontalWallSegments(~, y, innerL, innerR, doors, tt)
            % Build thin horizontal wall AABBs with gaps for door intervals.
            % doors: n×2 [xLeft xRight] per opening, sorted by xLeft.
            rows = zeros(0, 4);
            if isempty(doors)
                hw = (innerR - innerL) / 2;
                if hw > 1e-6
                    rows = [(innerL + innerR) / 2, y, hw, tt];
                end
                return;
            end
            [~, ord] = sort(doors(:, 1));
            doors = doors(ord, :);
            cursor = innerL;
            for ii = 1:size(doors, 1)
                dL = doors(ii, 1);
                dR = doors(ii, 2);
                if dL > cursor + 1e-6
                    c = (cursor + dL) / 2;
                    hw = (dL - cursor) / 2;
                    if hw > 1e-6
                        rows(end+1, :) = [c, y, hw, tt]; %#ok<AGROW>
                    end
                end
                cursor = max(cursor, dR);
            end
            if innerR > cursor + 1e-6
                c = (cursor + innerR) / 2;
                hw = (innerR - cursor) / 2;
                if hw > 1e-6
                    rows(end+1, :) = [c, y, hw, tt];
                end
            end
        end

        % ============================================================== %
        function assertDoorWidths(~, doors, minWidth)
            for ii = 1:size(doors, 1)
                w = doors(ii, 2) - doors(ii, 1);
                assert(w >= minWidth - 1e-6, ...
                    'HospitalEnv:doorWidth', ...
                    'Door width %.3f m is below minimum %.3f m.', w, minWidth);
            end
        end

        % ============================================================== %
        function chosen = selectIdleRobotForTask(obj, tx, ty)
            idleIdx = find(arrayfun(@(r) strcmp(r.Status,'idle'), obj.Robots));
            if isempty(idleIdx)
                chosen = -1;
                return;
            end

            switch obj.AssignmentPolicy
                case 'round_robin'
                    if obj.NextRoundRobinIdx > numel(obj.Robots)
                        obj.NextRoundRobinIdx = 1;
                    end
                    candidateOrder = [obj.NextRoundRobinIdx:numel(obj.Robots), ...
                        1:obj.NextRoundRobinIdx-1];
                    chosen = -1;
                    for c = candidateOrder
                        if strcmp(obj.Robots(c).Status, 'idle')
                            chosen = c;
                            obj.NextRoundRobinIdx = c + 1;
                            if obj.NextRoundRobinIdx > numel(obj.Robots)
                                obj.NextRoundRobinIdx = 1;
                            end
                            return;
                        end
                    end
                otherwise
                    dists = arrayfun(@(i) ...
                        norm([obj.Robots(i).X - tx, obj.Robots(i).Y - ty]), idleIdx);
                    [~, minI] = min(dists);
                    chosen = idleIdx(minI);
            end
        end
    end
end
