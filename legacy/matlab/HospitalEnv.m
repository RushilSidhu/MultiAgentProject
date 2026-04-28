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
        TotalNearMisses (1,1) double = 0
        DeadlockRecoveries (1,1) double = 0
    end

    properties (Access = private)
        Fig             % figure handle
        Ax              % world axes handle
        hObstacles      % patch handles for obstacles
        hRobots         % robot body patch handles
        hRobotHeadings  % robot heading line handles
        hNurseBodies    % nurse body patch handles
        hNurseCarts     % nurse cart patch handles
        hNurseHeadings  % nurse heading line handles
        hTasks          % scatter handle for pending tasks
        hLidar          % line handles for lidar rays
        hTitle          % text handle for metrics overlay (legacy, hidden)
        hLegend         % legend handle (legacy, hidden)
        hSpeedSlider    % slider UI handle
        hSpeedLabel     % text UI handle
        LidarVisible (1,1) logical = false
        ShowAllLidar (1,1) logical = false
        FocusedRobotIdx (1,1) double = 1
        RenderingEnabled (1,1) logical = true

        NextTaskID (1,1) double = 1
        AssignmentPolicy (1,:) char = 'random_item'
        NextRoundRobinIdx (1,1) double = 1
        PriorityPolicy (1,:) char = 'task_age'
        CriticalZones struct
        TaskSpaceTokens struct
        GoalApproachMap struct

        % Static geometry subsets (same row format as Obstacles) for rendering
        WallObstacles      (:,4) double = zeros(0,4)
        BedObstacles       (:,4) double = zeros(0,4)
        FurnitureObstacles (:,4) double = zeros(0,4)
        BaseObstacles      (:,4) double = zeros(0,4)
        DoorObstacles      (:,4) double = zeros(0,4)
        DoorOpen           (:,1) logical = false(0,1)

        hRoomFloors   % gobjects vector – translucent room floor patches
        hBedPatches   % gobjects vector
        hFurnPatches  % gobjects vector
        hDoorPatches  % gobjects vector
        hPoiMarkers   % scatter handle for task POI sites
        hRoomLabels   % text handles for room names

        % --- Visual rehaul: theming + per-robot identity ----------------
        StrategyName (1,:) char = ''
        RobotIdentityPalette (:,3) double = [
            0.20 0.78 0.78;
            0.95 0.55 0.20;
            0.55 0.85 0.30;
            0.85 0.40 0.85;
            0.30 0.65 1.00;
            0.95 0.85 0.30 ]
        RobotIdentityColors (:,3) double = zeros(0,3)

        % --- Layout containers ------------------------------------------
        TopPanel
        WorldPanel
        DashPanel

        % --- Top strip widgets ------------------------------------------
        hClockText
        hStrategyText
        hPauseBtn
        hLayerInfoText

        % --- World overlays (per-robot, pre-allocated lazily) -----------
        hPlannedFuture
        hPlannedPast
        hLeaderLines
        hRobotIdLabels
        hRobotStatusBadges
        hFocusRing
        hNurseIdLabels
        hZoneHalos
        hZoneLabels
        hTrails

        % --- Effects (transient, fade with time) ------------------------
        TaskFx struct = struct('id',{},'spawnTime',{},'hPulse',{})
        CollisionFx struct = struct('x',{},'y',{},'spawnTime',{},'hRing',{},'hText',{})

        % --- Dashboard widgets ------------------------------------------
        hDashAx
        hKpiTitle
        hKpiText
        hThroughputAx
        hThroughputLine
        hThroughputTitle
        hQueueTitle
        hQueueText
        hCardTitle
        hRobotCardPatches
        hRobotCardTexts
        hRobotCardStripes
        hLegendTitle
        hLegendText
        hEventTitle
        hEventLogText

        % --- Dashboard state --------------------------------------------
        ThroughputHistory (:,2) double = zeros(0,2)
        EventBuffer cell = {}
        MaxEventLogLines (1,1) double = 8

        % --- Toggles / UX state -----------------------------------------
        ShowPaths (1,1) logical = true
        ShowZones (1,1) logical = true
        ShowTrails (1,1) logical = true
        ShowBadges (1,1) logical = true
        ShowLeaderLines (1,1) logical = true
        ShowIds (1,1) logical = true
    end

    % ------------------------------------------------------------------ %
    methods
        % ============================================================== %
        function obj = HospitalEnv(width, height, assignmentPolicy, enableRendering, strategyName)
            if nargin >= 1, obj.Width  = width;  end
            if nargin >= 2, obj.Height = height; end
            if nargin >= 3 && ~isempty(assignmentPolicy)
                obj.AssignmentPolicy = assignmentPolicy;
            end
            if nargin >= 4
                obj.RenderingEnabled = logical(enableRendering);
            end
            if nargin >= 5 && ~isempty(strategyName)
                obj.StrategyName = char(strategyName);
            end

            obj.Robots = RobotAgent.empty(0,1);
            obj.Nurses = NurseAgent.empty(0,1);
            obj.Tasks  = struct('id',{},'x',{},'y',{},'assignedTo',{},...
                                'spawnTime',{},'poiName',{});

            obj.buildDefaultObstacles();
            obj.buildRooms();
            obj.buildPOIs();
            obj.buildCoordinationArtifacts();
            obj.ThroughputHistory = [0, 0];
            if obj.RenderingEnabled
                obj.initFigure();
            end
        end

        function setStrategyLabel(obj, name)
            obj.StrategyName = char(name);
            if obj.RenderingEnabled && ~isempty(obj.hStrategyText) && isvalid(obj.hStrategyText)
                obj.hStrategyText.String = sprintf('Strategy: %s', obj.StrategyName);
            end
        end

        % ============================================================== %
        function addRobot(obj, robot)
            robot.setStaticMap(obj.getStaticMapData());
            obj.Robots(end+1,1) = robot;
            paletteIdx = mod(numel(obj.Robots)-1, size(obj.RobotIdentityPalette,1)) + 1;
            obj.RobotIdentityColors(end+1,:) = obj.RobotIdentityPalette(paletteIdx,:);
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
                obj.pushEvent('Task %-3d spawn @ %s -> R%d', t.id, t.poiName, chosen);
            else
                fprintf('[%.2fs] Task %d spawned at %s (%.1f,%.1f) → QUEUED (no idle robot)\n', ...
                    obj.SimTime, t.id, t.poiName, t.x, t.y);
                obj.pushEvent('Task %-3d spawn @ %s -> QUEUED', t.id, t.poiName);
            end

            obj.Tasks(end+1) = t;
            obj.recordTaskSpawn(t.id, t.x, t.y);
        end

        % ============================================================== %
        function step(obj, dt)
            % Advance simulation by dt seconds.
            obj.SimTime = obj.SimTime + dt;
            obj.updateDoorStates();

            % 1. Update nurses (dynamic obstacles with local neighbor context)
            for k = 1:numel(obj.Nurses)
                selfN = obj.Nurses(k);
                otherIdx = [1:k-1, k+1:numel(obj.Nurses)];
                nCtx = zeros(numel(otherIdx), 3);
                for j = 1:numel(otherIdx)
                    q = obj.Nurses(otherIdx(j));
                    nCtx(j,:) = [q.X - selfN.X, q.Y - selfN.Y, q.Radius];
                end
                robCtx = zeros(numel(obj.Robots), 3);
                for j = 1:numel(obj.Robots)
                    rr = obj.Robots(j);
                    robCtx(j,:) = [rr.X - selfN.X, rr.Y - selfN.Y, rr.Radius];
                end
                obj.Nurses(k).step(dt, obj.Obstacles, obj.Width, obj.Height, nCtx, robCtx);
            end
            obj.updateDoorStates();

            % 2. Multi-agent coordinated robot update.
            priorityOrder = obj.computeRobotPriorityOrder();
            obj.updateCriticalZoneReservations(priorityOrder);
            for ord = 1:numel(priorityOrder)
                i = priorityOrder(ord);
                if ~obj.Robots(i).isOperational()
                    continue;
                end
                obj.Robots(i).setStaticMap(obj.getStaticMapData());
                obj.Robots(i).updatePlanningContext(obj.SimTime, dt);
                obj.enforceZoneAndTaskSpaceProtocols(i, ord, priorityOrder);
                otherRobots = obj.Robots([1:i-1, i+1:end]);
                otherRobots = otherRobots(arrayfun(@(r) r.isOperational(), otherRobots));
                lidarData   = obj.Robots(i).senseEnvironment(...
                    obj.Obstacles, obj.Nurses, otherRobots);
                [v, omega] = obj.Robots(i).calculateVelocity(...
                    obj.Robots(i).TargetX, obj.Robots(i).TargetY, lidarData);
                obj.Robots(i).move(v, omega, dt, obj.Obstacles, obj.Width, obj.Height);
                obj.updateDoorStates();
            end

            % 3. Collision detection
            obj.detectCollisions();
            obj.detectNearMisses();

            % 4. Task completion check
            obj.checkTaskCompletion();

            % 5. Reassign queued tasks if robots became idle
            obj.reassignQueuedTasks();
        end

        function tf = allRobotsOutOfOperation(obj)
            if isempty(obj.Robots)
                tf = true;
                return;
            end
            tf = ~any(arrayfun(@(r) r.isOperational(), obj.Robots));
        end

        function mapData = getStaticMapData(obj)
            % getStaticMapData  Known static geometry for global planners.
            mapData = struct( ...
                'width', obj.Width, ...
                'height', obj.Height, ...
                'obstacles', obj.Obstacles);
        end

        function tf = isStaticSegmentTraversable(obj, p0, p1, radius, sampleStep)
            % Query static traversability against known walls/doors geometry.
            if nargin < 5 || isempty(sampleStep)
                sampleStep = 0.2;
            end
            if nargin < 4 || isempty(radius)
                radius = 0.4;
            end
            seg = p1 - p0;
            segLen = norm(seg);
            if segLen < 1e-9
                tf = ~obj.positionInObstacle(p0(1), p0(2), radius);
                return;
            end
            n = max(2, ceil(segLen / sampleStep) + 1);
            ts = linspace(0, 1, n);
            tf = true;
            for ii = 1:numel(ts)
                p = p0 + ts(ii) * seg;
                if p(1) < radius || p(1) > obj.Width - radius || ...
                        p(2) < radius || p(2) > obj.Height - radius || ...
                        obj.positionInObstacle(p(1), p(2), radius)
                    tf = false;
                    return;
                end
            end
        end

        % ============================================================== %
        function render(obj)
            % Layered render using pre-allocated graphics objects.
            if ~obj.RenderingEnabled || isempty(obj.Ax) || ~isgraphics(obj.Ax)
                return;
            end

            obj.updateZoneHalos();
            obj.updatePlannedPaths();
            obj.updateLeaderLines();
            obj.updateLidar();
            obj.updateDoorPatches();
            obj.updateTrails();
            obj.updateNurses();
            obj.updateRobots();
            obj.updateFocusRing();
            obj.updateTasks();
            obj.updateTaskFx();
            obj.updateCollisionFx();

            % --- Top strip + dashboard --------------------------------
            if isvalid(obj.hClockText)
                obj.hClockText.String = sprintf('T = %5.1f s', obj.SimTime);
            end
            if isvalid(obj.hLayerInfoText)
                obj.hLayerInfoText.String = obj.layerInfoString();
            end
            obj.updateDashboard();

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

            obj.hSpeedSlider = uicontrol(obj.TopPanel, ...
                'Style', 'slider', ...
                'Units', 'normalized', ...
                'Position', [0.74 0.55 0.24 0.30], ...
                'Min', minSpeed, ...
                'Max', maxSpeed, ...
                'Value', initialSpeed, ...
                'Callback', @(src,~)obj.onSpeedSliderChanged(src));

            obj.hSpeedLabel = uicontrol(obj.TopPanel, ...
                'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.74 0.85 0.24 0.12], ...
                'String', sprintf('Speed: %.2fx', initialSpeed), ...
                'BackgroundColor', [0.08 0.10 0.16], ...
                'ForegroundColor', [0.92 0.96 1.00], ...
                'FontName','Consolas', 'FontSize', 9, ...
                'HorizontalAlignment', 'left');
        end

        % ============================================================== %
        function setLidarVisible(obj, tf)
            obj.LidarVisible = logical(tf);
            if obj.RenderingEnabled && ~isempty(obj.Ax) && isgraphics(obj.Ax)
                obj.updateLidar();
            end
        end

        % ============================================================== %
        function printMetrics(obj)
            fprintf('\n===== SIMULATION METRICS (T=%.2fs) =====\n', obj.SimTime);
            fprintf('  Total Collisions    : %d\n', obj.TotalCollisions);
            fprintf('  Near Misses         : %d\n', obj.TotalNearMisses);
            fprintf('  Deadlock Recoveries : %d\n', obj.DeadlockRecoveries);
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
            % 60 x 40 m ward: Row A (6 patient) / Hallway-1 / Row B (6 treatment) /
            % vertical connector (x=28..32) / Hallway-2 / Row C (6 clinical).
            % Path sanity: Hallway-1 (30,31) -> connector mouth y=29, y=22 -> Hallway-2 (30,20)
            % -> ICU-A door y=18 -> bedside_ICUA_1; all door openings >= 1.5 m (assert >= 1.2 m).
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

            % --- Y-bands (metres) ---------------------------------------
            yRowA0   = 33.00;   % patient wing (shortened)
            yHall1_0 = 29.00;   % Hallway-1
            yRowB0   = 22.00;   % treatment
            yHall2_0 = 18.00;   % Hallway-2
            yRowC0   = 10.00;   % clinical

            borders = [
                W/2,     tB/2,   W/2,     tB/2;
                W/2, H - tB/2,   W/2,     tB/2;
                tB/2,   H/2,     tB/2,    H/2;
                W - tB/2, H/2,   tB/2,    H/2;
            ];

            % Row A verticals (x = 10..50), y = yRowA0 .. innerT
            xvA = (10:10:50)';
            cyA = (yRowA0 + innerT) / 2;
            hhA = (innerT - yRowA0) / 2;
            vertRowA = [xvA, repmat(cyA, 5, 1), repmat(tt, 5, 1), repmat(hhA, 5, 1)];

            % Row B: left block dividers, right block dividers, connector sides
            cyB = (yRowB0 + yHall1_0) / 2;
            hhB = (yHall1_0 - yRowB0) / 2;
            vertRowB_L = [9.5; 18.75];
            vertRowB_R = [41.25; 50.5];
            vertRowB_L = [vertRowB_L, repmat(cyB, 2, 1), repmat(tt, 2, 1), repmat(hhB, 2, 1)];
            vertRowB_R = [vertRowB_R, repmat(cyB, 2, 1), repmat(tt, 2, 1), repmat(hhB, 2, 1)];
            % Connector verticals: clear passage interior x in (28, 32)
            vertConnL = [27.85, cyB, tt, hhB];
            vertConnR = [32.15, cyB, tt, hhB];

            % Row C verticals, y = yRowC0 .. yHall2_0
            xvC = (10:10:50)';
            cyC = (yRowC0 + yHall2_0) / 2;
            hhC = (yHall2_0 - yRowC0) / 2;
            vertRowC = [xvC, repmat(cyC, 5, 1), repmat(tt, 5, 1), repmat(hhC, 5, 1)];

            % --- Horizontal walls (doors 1.5 m unless noted) ------------
            doors_y33 = [
                4.25,  5.75;
                14.25, 15.75;
                24.25, 25.75;
                34.25, 35.75;
                44.25, 45.75;
                54.25, 55.75;
            ];
            obj.assertDoorWidths(doors_y33, 1.2);
            horiz_y33 = obj.horizontalWallSegments(yRowA0, innerL, innerR, doors_y33, tt);

            % Hallway-1 / Row B: room doors + 4 m connector mouth
            doors_y29 = [
                4.125,  5.625;
                13.375, 14.875;
                22.625, 24.125;
                28.00,  32.00;
                35.875, 37.375;
                45.125, 46.625;
                54.375, 55.875;
            ];
            obj.assertDoorWidths(doors_y29, 1.2);
            horiz_y29 = obj.horizontalWallSegments(yHall1_0, innerL, innerR, doors_y29, tt);

            % Row B / Hallway-2: solid except connector mouth
            doors_y22 = [28.00, 32.00];
            obj.assertDoorWidths(doors_y22, 1.2);
            horiz_y22 = obj.horizontalWallSegments(yRowB0, innerL, innerR, doors_y22, tt);

            doors_y18 = doors_y33; % same centres as Row A doors
            obj.assertDoorWidths(doors_y18, 1.2);
            horiz_y18 = obj.horizontalWallSegments(yHall2_0, innerL, innerR, doors_y18, tt);

            obj.WallObstacles = [borders; vertRowA; vertRowB_L; vertRowB_R; vertConnL; vertConnR; ...
                vertRowC; horiz_y33; horiz_y29; horiz_y22; horiz_y18];
            % Door intervals are represented as permanent openings in the
            % wall geometry (no dynamic door collider panels).
            obj.DoorObstacles = zeros(0, 4);
            obj.DoorOpen = false(0, 1);

            % --- Beds ---------------------------------------------------
            bedHW = 1.2; bedHH = 0.5;
            patCx = (5:10:55)';
            patientBeds = [patCx, repmat(38.5, 6, 1), repmat(bedHW, 6, 1), repmat(bedHH, 6, 1)];
            examTableB = [55, 24.5, 1.0, 0.6];
            orTableB   = [35, 24.5, 1.0, 0.6];
            icuA_cx = [32.5; 37.5];
            icuB_cx = [39.5; 43.5];
            icuBeds = [icuA_cx, repmat(13.5, 2, 1), repmat(0.6, 2, 1), repmat(1.2, 2, 1); ...
                       icuB_cx, repmat(13.5, 2, 1), repmat(0.6, 2, 1), repmat(1.2, 2, 1)];
            erBed = [55, 13.5, 1.0, 0.6];
            obj.BedObstacles = [patientBeds; examTableB; orTableB; icuBeds; erBed];

            % --- Furniture ----------------------------------------------
            labBench   = [5,   27.0, 1.5, 0.5];
            radScan    = [15,  24.5, 1.0, 0.8];
            counter    = [5,   11.5, 4.0, 0.5];
            pharmShel  = [15,  11.5, 2.5, 0.5];
            suppShel   = [25,  11.5, 2.5, 0.5];
            obj.FurnitureObstacles = [labBench; radScan; counter; pharmShel; suppShel];

            obj.BaseObstacles = [obj.WallObstacles; obj.BedObstacles; obj.FurnitureObstacles];
            obj.updateEffectiveObstacles();
        end

        % ============================================================== %
        function buildRooms(obj)
            cPatient   = [0.10 0.18 0.30];
            cTreatment = [0.10 0.24 0.30];
            cICU       = [0.30 0.12 0.12];
            cNurse     = [0.10 0.28 0.18];
            cPharm     = [0.30 0.24 0.10];
            cSupply    = [0.18 0.20 0.28];
            cCorridor  = [0.11 0.12 0.16];
            cER        = [0.32 0.16 0.12];

            R = struct('name',{},'type',{},'xmin',{},'xmax',{},'ymin',{},'ymax',{}, ...
                'floorColor',{},'labelX',{},'labelY',{});

            % Row A — 6 patient rooms (interior flush to walls at x = 10,20,...,50)
            R(end+1) = struct('name','P101','type','patient', ...
                'xmin',0.25,'xmax',9.85,'ymin',33,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',5.05,'labelY',37.5);
            R(end+1) = struct('name','P102','type','patient', ...
                'xmin',10.15,'xmax',19.85,'ymin',33,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',15,'labelY',37.5);
            R(end+1) = struct('name','P103','type','patient', ...
                'xmin',20.15,'xmax',29.85,'ymin',33,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',25,'labelY',37.5);
            R(end+1) = struct('name','P104','type','patient', ...
                'xmin',30.15,'xmax',39.85,'ymin',33,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',35,'labelY',37.5);
            R(end+1) = struct('name','P105','type','patient', ...
                'xmin',40.15,'xmax',49.85,'ymin',33,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',45,'labelY',37.5);
            R(end+1) = struct('name','P106','type','patient', ...
                'xmin',50.15,'xmax',59.75,'ymin',33,'ymax',39.75, ...
                'floorColor',cPatient,'labelX',55,'labelY',37.5);

            R(end+1) = struct('name','Hallway-1','type','corridor', ...
                'xmin',0.25,'xmax',59.75,'ymin',29,'ymax',33, ...
                'floorColor',cCorridor,'labelX',8,'labelY',31);
            R(end+1) = struct('name','Hallway-2','type','corridor', ...
                'xmin',0.25,'xmax',59.75,'ymin',18,'ymax',22, ...
                'floorColor',cCorridor,'labelX',8,'labelY',20);

            % Row B — treatment (6) + connector strip (walls at 9.5, 18.75, 27.85 / 32.15, 41.25, 50.5)
            R(end+1) = struct('name','LAB','type','treatment', ...
                'xmin',0.25,'xmax',9.35,'ymin',22,'ymax',29, ...
                'floorColor',cTreatment,'labelX',4.8,'labelY',25.5);
            R(end+1) = struct('name','RAD','type','treatment', ...
                'xmin',9.65,'xmax',18.60,'ymin',22,'ymax',29, ...
                'floorColor',cTreatment,'labelX',14.1,'labelY',25.5);
            R(end+1) = struct('name','PT','type','treatment', ...
                'xmin',18.90,'xmax',27.85,'ymin',22,'ymax',29, ...
                'floorColor',cTreatment,'labelX',23.4,'labelY',25.5);
            R(end+1) = struct('name','Connector','type','connector', ...
                'xmin',28.15,'xmax',31.85,'ymin',22,'ymax',29, ...
                'floorColor',cCorridor,'labelX',30,'labelY',25.5);
            R(end+1) = struct('name','OR','type','treatment', ...
                'xmin',32.15,'xmax',41.10,'ymin',22,'ymax',29, ...
                'floorColor',cTreatment,'labelX',36.6,'labelY',25.5);
            R(end+1) = struct('name','RECOVERY','type','treatment', ...
                'xmin',41.40,'xmax',50.35,'ymin',22,'ymax',29, ...
                'floorColor',cTreatment,'labelX',45.9,'labelY',25.5);
            R(end+1) = struct('name','EXAM','type','treatment', ...
                'xmin',50.65,'xmax',59.75,'ymin',22,'ymax',29, ...
                'floorColor',cTreatment,'labelX',55.2,'labelY',25.5);

            % Row C — clinical (6), walls at x = 10..50
            R(end+1) = struct('name','NURSE','type','nurse', ...
                'xmin',0.25,'xmax',9.85,'ymin',10,'ymax',18, ...
                'floorColor',cNurse,'labelX',5.05,'labelY',14);
            R(end+1) = struct('name','PHARMACY','type','pharmacy', ...
                'xmin',10.15,'xmax',19.85,'ymin',10,'ymax',18, ...
                'floorColor',cPharm,'labelX',15,'labelY',14);
            R(end+1) = struct('name','SUPPLY','type','supply', ...
                'xmin',20.15,'xmax',29.85,'ymin',10,'ymax',18, ...
                'floorColor',cSupply,'labelX',25,'labelY',14);
            R(end+1) = struct('name','ICU-A','type','icu', ...
                'xmin',30.15,'xmax',39.85,'ymin',10,'ymax',18, ...
                'floorColor',cICU,'labelX',35,'labelY',14);
            R(end+1) = struct('name','ICU-B','type','icu', ...
                'xmin',40.15,'xmax',49.85,'ymin',10,'ymax',18, ...
                'floorColor',cICU,'labelX',45,'labelY',14);
            R(end+1) = struct('name','ER','type','er', ...
                'xmin',50.15,'xmax',59.75,'ymin',10,'ymax',18, ...
                'floorColor',cER,'labelX',55,'labelY',14);

            obj.Rooms = R;
            nReal = sum(~ismember({obj.Rooms.type}, {'corridor', 'connector'}));
            assert(nReal == 18, 'HospitalEnv:buildRooms', ...
                'Expected 18 clinical/patient rooms, got %d.', nReal);
        end

        % ============================================================== %
        function buildPOIs(obj)
            % Delivery targets + nurse tour stops (world frame, metres).
            names = { ...
                'bedside_P101'; 'bedside_P102'; 'bedside_P103'; 'bedside_P104'; ...
                'bedside_P105'; 'bedside_P106'; ...
                'bedside_ICUA_1'; 'bedside_ICUA_2'; 'bedside_ICUB_1'; 'bedside_ICUB_2'; ...
                'lab_pickup'; 'rad_table'; 'pt_station'; 'or_table'; 'recovery_bed'; 'exam_table'; ...
                'pharmacy_pickup'; 'supply_pickup'; 'er_bay'; ...
                'nurse_desk'; 'corridor1_mid'; 'corridor2_mid' };
            xs = [5; 15; 25; 35; 45; 55; ...
                32.5; 37.5; 39.5; 43.5; ...
                5; 15; 25; 35; 45; 55; ...
                15; 25; 55; ...
                5; 30; 30];
            ys = [36; 36; 36; 36; 36; 36; ...
                15.5; 15.5; 15.5; 15.5; ...
                26; 25.5; 25.5; 25.5; 25.5; 25.5; ...
                12.5; 12.5; 12.5; ...
                12.5; 31; 20];
            rms = { 'P101'; 'P102'; 'P103'; 'P104'; 'P105'; 'P106'; ...
                'ICU-A'; 'ICU-A'; 'ICU-B'; 'ICU-B'; ...
                'LAB'; 'RAD'; 'PT'; 'OR'; 'RECOVERY'; 'EXAM'; ...
                'PHARMACY'; 'SUPPLY'; 'ER'; ...
                'NURSE'; 'Hallway-1'; 'Hallway-2' };
            tt = [true(19,1); false(3,1)];
            ns = true(22,1);

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

        function buildCoordinationArtifacts(obj)
            % Reservable bottlenecks used for time-window entry control.
            z = struct('name', {}, 'center', {}, 'radius', {}, ...
                'reservedBy', {}, 'reservedUntil', {});
            z(end+1) = struct('name', 'connector_north', 'center', [30.0, 28.8], ...
                'radius', 2.2, 'reservedBy', -1, 'reservedUntil', -Inf);
            z(end+1) = struct('name', 'connector_south', 'center', [30.0, 22.2], ...
                'radius', 2.2, 'reservedBy', -1, 'reservedUntil', -Inf);
            z(end+1) = struct('name', 'connector_core', 'center', [30.0, 25.5], ...
                'radius', 2.0, 'reservedBy', -1, 'reservedUntil', -Inf);
            obj.CriticalZones = z;

            % Task-space occupancy tokens, one token per delivery POI by default.
            t = struct('name', {}, 'capacity', {}, 'holders', {});
            taskPOIs = obj.POIs([obj.POIs.taskTarget]);
            for k = 1:numel(taskPOIs)
                tkName = matlab.lang.makeValidName(taskPOIs(k).name);
                t(end+1) = struct('name', tkName, 'capacity', 1, 'holders', zeros(0,1)); %#ok<AGROW>
            end
            obj.TaskSpaceTokens = t;

            % Per-target approach waypoint: snap to nearest hallway if target inside room.
            m = struct();
            for k = 1:numel(taskPOIs)
                q = taskPOIs(k);
                approachY = q.y;
                if q.y > 33 || (q.y >= 22 && q.y <= 29)
                    approachY = 31.0;
                elseif q.y < 18
                    approachY = 20.0;
                end
                m.(matlab.lang.makeValidName(q.name)) = [q.x, approachY];
            end
            obj.GoalApproachMap = m;
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
            % Theme + figure container
            bgFig = [0.05 0.06 0.10];
            obj.Fig = figure('Name','Hospital Simulation', ...
                'Color', bgFig, ...
                'Units','normalized','Position',[0.02 0.05 0.94 0.88], ...
                'KeyPressFcn', @(~,evt)obj.onFigureKeyPress(evt));

            % Three-zone layout via uipanels (top strip / world / dashboard)
            obj.TopPanel = uipanel(obj.Fig, 'Units','normalized', ...
                'Position',[0 0.92 1 0.08], ...
                'BackgroundColor', [0.08 0.10 0.16], ...
                'BorderType','none');
            obj.WorldPanel = uipanel(obj.Fig, 'Units','normalized', ...
                'Position',[0 0 0.66 0.92], ...
                'BackgroundColor', bgFig, ...
                'BorderType','none');
            obj.DashPanel = uipanel(obj.Fig, 'Units','normalized', ...
                'Position',[0.66 0 0.34 0.92], ...
                'BackgroundColor', [0.07 0.08 0.13], ...
                'BorderType','line', ...
                'HighlightColor',[0.18 0.22 0.32]);

            obj.setupTopStrip();
            obj.setupWorldAxes();
            obj.setupDashboard();
        end

        % ============================================================== %
        function setupTopStrip(obj)
            txtCol = [0.92 0.96 1.00];
            uicontrol(obj.TopPanel, 'Style','text', ...
                'Units','normalized','Position',[0.005 0.10 0.18 0.80], ...
                'String','HOSPITAL MULTI-AGENT SIM', ...
                'BackgroundColor',[0.08 0.10 0.16], ...
                'ForegroundColor',[0.55 0.85 1.00], ...
                'FontWeight','bold','FontSize',12, ...
                'HorizontalAlignment','left');

            obj.hClockText = uicontrol(obj.TopPanel, 'Style','text', ...
                'Units','normalized','Position',[0.19 0.10 0.18 0.80], ...
                'String','T = 0.0 s', ...
                'BackgroundColor',[0.08 0.10 0.16], ...
                'ForegroundColor',txtCol, ...
                'FontName','Consolas','FontSize',16,'FontWeight','bold', ...
                'HorizontalAlignment','left');

            obj.hStrategyText = uicontrol(obj.TopPanel, 'Style','text', ...
                'Units','normalized','Position',[0.37 0.10 0.30 0.80], ...
                'String', sprintf('Strategy: %s', obj.StrategyName), ...
                'BackgroundColor',[0.08 0.10 0.16], ...
                'ForegroundColor',[0.80 0.86 0.96], ...
                'FontSize',11, ...
                'HorizontalAlignment','left');

            obj.hLayerInfoText = uicontrol(obj.TopPanel, 'Style','text', ...
                'Units','normalized','Position',[0.67 0.06 0.32 0.40], ...
                'String', obj.layerInfoString(), ...
                'BackgroundColor',[0.08 0.10 0.16], ...
                'ForegroundColor',[0.55 0.66 0.82], ...
                'FontName','Consolas','FontSize',9, ...
                'HorizontalAlignment','left');

            obj.hPauseBtn = uicontrol(obj.TopPanel, 'Style','pushbutton', ...
                'Units','normalized','Position',[0.67 0.50 0.06 0.42], ...
                'String','LIDAR', ...
                'BackgroundColor',[0.14 0.18 0.26], ...
                'ForegroundColor',txtCol, ...
                'FontWeight','bold', ...
                'Callback', @(~,~)obj.toggleLidarFromButton());
        end

        % ============================================================== %
        function setupWorldAxes(obj)
            obj.Ax = axes('Parent', obj.WorldPanel, ...
                'Units','normalized','Position',[0.04 0.05 0.94 0.92], ...
                'Color',[0.07 0.08 0.13],...
                'XColor',[0.45 0.50 0.62],'YColor',[0.45 0.50 0.62], ...
                'FontSize',8, 'DataAspectRatio',[1 1 1]);
            hold(obj.Ax,'on');
            axis(obj.Ax, [0 obj.Width 0 obj.Height]);
            grid(obj.Ax,'on');
            obj.Ax.GridColor = [0.18 0.20 0.28];
            obj.Ax.GridAlpha = 0.30;
            obj.Ax.Box = 'on';

            % --- Room floor tint (below walls) ------------------------
            nR = numel(obj.Rooms);
            obj.hRoomFloors = gobjects(nR, 1);
            for ri = 1:nR
                Rm = obj.Rooms(ri);
                if strcmp(Rm.type, 'corridor') || strcmp(Rm.type, 'connector')
                    fa = 0.10;
                else
                    fa = 0.32;
                end
                col = Rm.floorColor;
                obj.hRoomFloors(ri) = patch(obj.Ax, ...
                    [Rm.xmin Rm.xmax Rm.xmax Rm.xmin Rm.xmin], ...
                    [Rm.ymin Rm.ymin Rm.ymax Rm.ymax Rm.ymin], ...
                    col, 'FaceAlpha', fa, 'EdgeColor', 'none');
            end

            % --- Structural walls -------------------------------------
            for i = 1:size(obj.WallObstacles,1)
                cx = obj.WallObstacles(i,1); cy = obj.WallObstacles(i,2);
                hw = obj.WallObstacles(i,3); hh = obj.WallObstacles(i,4);
                patch(obj.Ax, ...
                    cx+[-hw hw hw -hw -hw], ...
                    cy+[-hh -hh hh hh -hh], ...
                    [0.42 0.46 0.56], 'EdgeColor', [0.62 0.66 0.78], 'LineWidth', 0.6);
            end

            % --- Sliding door panels -----------------------------------
            obj.hDoorPatches = gobjects(0, 1);

            % --- Beds (desaturated so robots stay dominant) -----------
            nb = size(obj.BedObstacles,1);
            obj.hBedPatches = gobjects(nb, 1);
            for i = 1:nb
                cx = obj.BedObstacles(i,1); cy = obj.BedObstacles(i,2);
                hw = obj.BedObstacles(i,3); hh = obj.BedObstacles(i,4);
                obj.hBedPatches(i) = patch(obj.Ax, ...
                    cx+[-hw hw hw -hw -hw], ...
                    cy+[-hh -hh hh hh -hh], ...
                    [0.55 0.60 0.72], 'EdgeColor', [0.42 0.48 0.60], 'LineWidth', 0.4, ...
                    'FaceAlpha', 0.85);
            end

            % --- Furniture --------------------------------------------
            nf = size(obj.FurnitureObstacles,1);
            obj.hFurnPatches = gobjects(nf, 1);
            for i = 1:nf
                cx = obj.FurnitureObstacles(i,1); cy = obj.FurnitureObstacles(i,2);
                hw = obj.FurnitureObstacles(i,3); hh = obj.FurnitureObstacles(i,4);
                obj.hFurnPatches(i) = patch(obj.Ax, ...
                    cx+[-hw hw hw -hw -hw], ...
                    cy+[-hh -hh hh hh -hh], ...
                    [0.30 0.34 0.42], 'EdgeColor', [0.44 0.48 0.58], 'LineWidth', 0.4);
            end

            % --- Task POI markers (clearer than before) ---------------
            ttMask = arrayfun(@(q) q.taskTarget, obj.POIs);
            xp = [obj.POIs(ttMask).x];
            yp = [obj.POIs(ttMask).y];
            obj.hPoiMarkers = scatter(obj.Ax, xp, yp, 36, ...
                'Marker','o', 'MarkerEdgeColor', [0.55 0.72 0.92], ...
                'MarkerFaceColor', 'none', 'LineWidth', 0.7, ...
                'MarkerEdgeAlpha', 0.65);

            % --- Critical zone halos (pre-allocated) ------------------
            nZ = numel(obj.CriticalZones);
            obj.hZoneHalos = gobjects(nZ, 1);
            obj.hZoneLabels = gobjects(nZ, 1);
            for zi = 1:nZ
                cz = obj.CriticalZones(zi);
                th = linspace(0, 2*pi, 64);
                xc = cz.center(1) + cz.radius * cos(th);
                yc = cz.center(2) + cz.radius * sin(th);
                obj.hZoneHalos(zi) = patch(obj.Ax, xc, yc, [0.30 0.85 0.55], ...
                    'FaceAlpha', 0.10, ...
                    'EdgeColor', [0.30 0.85 0.55], ...
                    'LineWidth', 1.0, ...
                    'LineStyle', '--');
                obj.hZoneLabels(zi) = text(obj.Ax, ...
                    cz.center(1), cz.center(2) + cz.radius + 0.30, ...
                    sprintf('%s : free', strrep(cz.name,'_','-')), ...
                    'Color', [0.55 0.78 0.62], ...
                    'FontSize', 7, 'FontName','Consolas', ...
                    'HorizontalAlignment','center', 'VerticalAlignment','bottom');
            end

            % --- Room labels -------------------------------------------
            obj.hRoomLabels = gobjects(nR, 1);
            for ri = 1:nR
                Rm = obj.Rooms(ri);
                obj.hRoomLabels(ri) = text(obj.Ax, Rm.labelX, Rm.labelY, Rm.name, ...
                    'Color', [0.74 0.84 0.96], 'FontSize', 8, ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                    'FontWeight', 'bold', 'FontName','Consolas');
            end

            % Placeholder containers (per-entity handles allocated lazily)
            obj.hRobots = gobjects(0);
            obj.hRobotHeadings = gobjects(0);
            obj.hPlannedFuture = gobjects(0);
            obj.hPlannedPast = gobjects(0);
            obj.hLeaderLines = gobjects(0);
            obj.hRobotIdLabels = gobjects(0);
            obj.hRobotStatusBadges = gobjects(0);
            obj.hNurseBodies = gobjects(0);
            obj.hNurseCarts = gobjects(0);
            obj.hNurseHeadings = gobjects(0);
            obj.hNurseIdLabels = gobjects(0);
            obj.hLidar = gobjects(0);
            obj.hTrails = {};

            % Focus ring (idle until first robot exists)
            obj.hFocusRing = line(obj.Ax, NaN, NaN, ...
                'Color', [0.95 0.95 0.55], 'LineWidth', 1.4, ...
                'LineStyle', ':', 'Visible', 'off');

            % Task scatter (pending, unassigned tasks only)
            obj.hTasks = scatter(obj.Ax, NaN, NaN, 130, 'Marker','pentagram', ...
                'MarkerEdgeColor',[1.0 0.45 0.32], ...
                'MarkerFaceColor',[1.0 0.78 0.30], ...
                'LineWidth', 1.2, 'Visible','off');

            % Legacy title / legend hidden but kept to satisfy old refs
            obj.hLegend = legend(obj.Ax, 'off');
            obj.hTitle = title(obj.Ax, '', 'Color','w','FontSize',9, ...
                'FontName','Consolas','Visible','off');
            xlabel(obj.Ax,''); ylabel(obj.Ax,'');
        end

        % ============================================================== %
        function setupDashboard(obj)
            % Single invisible axes covering the whole dashboard panel for
            % free placement of headings, KPIs, sparkline, queue, log.
            obj.hDashAx = axes('Parent', obj.DashPanel, ...
                'Units','normalized','Position',[0 0 1 1], ...
                'Color', [0.07 0.08 0.13], ...
                'XColor','none','YColor','none', ...
                'XLim',[0 1],'YLim',[0 1], ...
                'NextPlot','add');
            try, obj.hDashAx.Toolbar.Visible = 'off'; catch, end
            try, disableDefaultInteractivity(obj.hDashAx); catch, end

            txtCol = [0.92 0.96 1.00];
            dimCol = [0.62 0.70 0.82];

            % --- KPI block (top of dashboard) -------------------------
            obj.hKpiTitle = text(obj.hDashAx, 0.04, 0.965, 'KPIs', ...
                'Color',[0.55 0.85 1.00],'FontSize',11,'FontWeight','bold');
            obj.hKpiText = text(obj.hDashAx, 0.04, 0.86, ...
                obj.kpiTextString(), ...
                'Color', txtCol, 'FontName','Consolas','FontSize',10, ...
                'VerticalAlignment','top','HorizontalAlignment','left');

            % --- Throughput sparkline ---------------------------------
            obj.hThroughputTitle = text(obj.hDashAx, 0.04, 0.74, ...
                'Throughput  (tasks completed)', ...
                'Color',[0.55 0.85 1.00],'FontSize',10,'FontWeight','bold');
            obj.hThroughputAx = axes('Parent', obj.DashPanel, ...
                'Units','normalized','Position',[0.06 0.62 0.90 0.10], ...
                'Color',[0.10 0.12 0.18], ...
                'XColor',[0.45 0.52 0.66],'YColor',[0.45 0.52 0.66], ...
                'FontSize',7,'GridAlpha',0.20);
            hold(obj.hThroughputAx,'on');
            obj.hThroughputAx.XLabel.String = '';
            obj.hThroughputAx.YLabel.String = '';
            try, obj.hThroughputAx.Toolbar.Visible = 'off'; catch, end
            try, disableDefaultInteractivity(obj.hThroughputAx); catch, end
            obj.hThroughputLine = line(obj.hThroughputAx, 0, 0, ...
                'Color',[0.30 0.85 0.55],'LineWidth',1.6);

            % --- Robot status cards -----------------------------------
            obj.hCardTitle = text(obj.hDashAx, 0.04, 0.585, 'Robots', ...
                'Color',[0.55 0.85 1.00],'FontSize',10,'FontWeight','bold');
            obj.hRobotCardPatches = gobjects(0);
            obj.hRobotCardTexts = gobjects(0);
            obj.hRobotCardStripes = gobjects(0);

            % --- Active task queue ------------------------------------
            obj.hQueueTitle = text(obj.hDashAx, 0.04, 0.355, 'Queue', ...
                'Color',[0.55 0.85 1.00],'FontSize',10,'FontWeight','bold');
            obj.hQueueText = text(obj.hDashAx, 0.04, 0.335, '(empty)', ...
                'Color', dimCol, 'FontName','Consolas','FontSize',9, ...
                'VerticalAlignment','top','HorizontalAlignment','left');

            % --- Compact legend ---------------------------------------
            obj.hLegendTitle = text(obj.hDashAx, 0.04, 0.225, 'Legend', ...
                'Color',[0.55 0.85 1.00],'FontSize',10,'FontWeight','bold');
            obj.hLegendText = text(obj.hDashAx, 0.04, 0.205, ...
                obj.legendTextString(), ...
                'Color', dimCol, 'FontName','Consolas','FontSize',8, ...
                'VerticalAlignment','top','HorizontalAlignment','left');

            % --- Event log --------------------------------------------
            obj.hEventTitle = text(obj.hDashAx, 0.04, 0.090, 'Events', ...
                'Color',[0.55 0.85 1.00],'FontSize',10,'FontWeight','bold');
            obj.hEventLogText = text(obj.hDashAx, 0.04, 0.075, '(no events yet)', ...
                'Color', dimCol, 'FontName','Consolas','FontSize',8, ...
                'VerticalAlignment','top','HorizontalAlignment','left');
        end

        % ============================================================== %
        function detectCollisions(obj)
            % Robot-involved collisions are terminal for the robot.
            nRobots = numel(obj.Robots);
            nNurses = numel(obj.Nurses);

            % Robot-robot
            for i = 1:nRobots-1
                for j = i+1:nRobots
                    ri = obj.Robots(i);
                    rj = obj.Robots(j);
                    if ~ri.isOperational() || ~rj.isOperational()
                        continue;
                    end
                    dist = norm([ri.X-rj.X, ri.Y-rj.Y]);
                    if dist < (ri.Radius + rj.Radius)
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] Robot %d vs Robot %d  dist=%.3f -> both out of operation\n',...
                            obj.SimTime, i, j, dist);
                        obj.pushEvent('R%d <-> R%d  COLLISION', i, j);
                        midX = (ri.X + rj.X) / 2;
                        midY = (ri.Y + rj.Y) / 2;
                        obj.recordCollisionFx(midX, midY);
                        obj.retireRobot(i, sprintf('Robot collision with Robot %d', j));
                        obj.retireRobot(j, sprintf('Robot collision with Robot %d', i));
                    end
                end
            end

            % Robot-nurse
            for i = 1:nRobots
                if ~obj.Robots(i).isOperational()
                    continue;
                end
                for j = 1:nNurses
                    r = obj.Robots(i);
                    n = obj.Nurses(j);
                    dist = norm([r.X-n.X, r.Y-n.Y]);
                    if dist < (r.Radius + n.Radius)
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] Robot %d vs Nurse %d  dist=%.3f -> robot out of operation\n',...
                            obj.SimTime, i, j, dist);
                        obj.pushEvent('R%d  hit  N%d', i, j);
                        midX = (r.X + n.X) / 2;
                        midY = (r.Y + n.Y) / 2;
                        obj.recordCollisionFx(midX, midY);
                        obj.retireRobot(i, sprintf('Collision with Nurse %d', j));
                        break;
                    end
                end
            end

            % Entity-wall (AABB) collisions
            for i = 1:nRobots
                e = obj.Robots(i);
                if ~e.isOperational()
                    continue;
                end
                for w = 1:size(obj.Obstacles,1)
                    if obj.circleAABBOverlap(e.X, e.Y, e.Radius, obj.Obstacles(w,:))
                        obj.TotalCollisions = obj.TotalCollisions + 1;
                        fprintf('[COLLISION %.2fs] Robot %d hit obstacle %d -> out of operation\n',...
                            obj.SimTime, i, w);
                        obj.pushEvent('R%d  hit obstacle %d', i, w);
                        obj.recordCollisionFx(e.X, e.Y);
                        obj.retireRobot(i, sprintf('Collision with obstacle %d', w));
                        break;
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
                        obj.pushEvent('R%d done T%-3d in %.1fs', i, taskId, elapsed);
                        r.clearTarget();
                        obj.releaseTaskTokenByRobot(i);
                        obj.removeTask(taskId);
                        obj.recordCompletion();
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
                        obj.pushEvent('Task %-3d -> R%d (assign)', obj.Tasks(ti).id, chosen);
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
        function updateDoorStates(obj)
            % Doors are modeled as always-open gaps; no dynamic state.
            obj.DoorOpen = false(size(obj.DoorObstacles, 1), 1);
            obj.updateEffectiveObstacles();
        end

        % ============================================================== %
        function updateEffectiveObstacles(obj)
            obj.Obstacles = obj.BaseObstacles;
        end

        % ============================================================== %
        function updateDoorPatches(obj)
            % No door panels to update (doorways are permanent openings).
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
                case 'out_of_operation', c = [0.36 0.36 0.40];
                otherwise,    c = [0.72 0.72 0.72];
            end
        end

        % ============================================================== %
        function [bodyX, bodyY, cartX, cartY] = nurseGlyph(~, nurse)
            th = linspace(0, 2*pi, 28);
            bodyR = 0.26;
            bodyX = nurse.X + bodyR*cos(th);
            bodyY = nurse.Y + bodyR*sin(th);

            if nurse.HasCart
                u = [cos(nurse.Theta), sin(nurse.Theta)];
                p = [-u(2), u(1)];

                cartLen = 0.56;
                cartWid = 0.34;
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
        function [xRays, yRays] = packLidarHitSegments(~, robot)
            n = numel(robot.LidarRanges);
            xRays = nan(1, 3*n);
            yRays = nan(1, 3*n);
            if n == 0
                return;
            end

            worldAngles = robot.LidarAngles + robot.Theta;
            endX = robot.X + robot.LidarRanges .* cos(worldAngles);
            endY = robot.Y + robot.LidarRanges .* sin(worldAngles);

            idx0 = 1:3:(3*n);
            idx1 = 2:3:(3*n);
            xRays(idx0) = robot.X;
            xRays(idx1) = endX;
            yRays(idx0) = robot.Y;
            yRays(idx1) = endY;
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

        function onFigureKeyPress(obj, evt)
            switch lower(evt.Key)
                case 'tab'
                    nR = numel(obj.Robots);
                    if nR > 0
                        obj.FocusedRobotIdx = mod(obj.FocusedRobotIdx, nR) + 1;
                    end
                case 'l'
                    obj.LidarVisible = ~obj.LidarVisible;
                    obj.updateLidar();
                case 'a'
                    obj.ShowAllLidar = ~obj.ShowAllLidar;
                    obj.updateLidar();
                case 'p'
                    obj.ShowPaths = ~obj.ShowPaths;
                case 'g'
                    obj.ShowLeaderLines = ~obj.ShowLeaderLines;
                case 'z'
                    obj.ShowZones = ~obj.ShowZones;
                    for zi = 1:numel(obj.hZoneHalos)
                        if isvalid(obj.hZoneHalos(zi))
                            set(obj.hZoneHalos(zi), 'Visible', obj.boolToOnOff(obj.ShowZones));
                        end
                        if isvalid(obj.hZoneLabels(zi))
                            set(obj.hZoneLabels(zi), 'Visible', obj.boolToOnOff(obj.ShowZones));
                        end
                    end
                case 't'
                    obj.ShowTrails = ~obj.ShowTrails;
                    for ri = 1:numel(obj.hTrails)
                        if ~isempty(obj.hTrails{ri}) && isvalid(obj.hTrails{ri})
                            set(obj.hTrails{ri}, 'Visible', obj.boolToOnOff(obj.ShowTrails));
                        end
                    end
                case 'b'
                    obj.ShowBadges = ~obj.ShowBadges;
                case 'i'
                    obj.ShowIds = ~obj.ShowIds;
            end
        end

        function [xPoly, yPoly] = robotGlyph(~, robot)
            % Oriented rounded rectangle: clearer chassis footprint than circle.
            len = 2.4 * robot.Radius;
            wid = 1.7 * robot.Radius;
            rad = min(0.24 * len, 0.45 * wid);
            [bx, by] = HospitalEnv.roundedRectLocal(len, wid, rad, 6);
            c = cos(robot.Theta);
            s = sin(robot.Theta);
            xPoly = robot.X + c * bx - s * by;
            yPoly = robot.Y + s * bx + c * by;
        end

        % ============================================================== %
        function rows = doorPanels(~, y, doors, tt)
            rows = zeros(0, 4);
            if isempty(doors)
                return;
            end
            rows = [(doors(:,1) + doors(:,2)) / 2, ...
                repmat(y, size(doors,1), 1), ...
                (doors(:,2) - doors(:,1)) / 2, ...
                repmat(tt, size(doors,1), 1)];
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
                case 'random_item'
                    chosen = idleIdx(randi(numel(idleIdx)));
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
                case 'nearest_idle'
                    dists = arrayfun(@(i) ...
                        norm([obj.Robots(i).X - tx, obj.Robots(i).Y - ty]), idleIdx);
                    [~, minI] = min(dists);
                    chosen = idleIdx(minI);
                otherwise
                    % Unknown policy fallback: use random item-holder style dispatch.
                    chosen = idleIdx(randi(numel(idleIdx)));
            end
        end

        function retireRobot(obj, robotIdx, reason)
            r = obj.Robots(robotIdx);
            if ~r.isOperational()
                return;
            end
            taskId = r.CurrentTaskID;
            r.retire();
            obj.releaseTaskTokenByRobot(robotIdx);
            fprintf('[ROBOT DOWN %.2fs] Robot %d retired (%s)\n', ...
                obj.SimTime, robotIdx, reason);
            obj.pushEvent('R%d  DOWN  (%s)', robotIdx, reason);

            if taskId < 0
                return;
            end

            for ti = 1:numel(obj.Tasks)
                if obj.Tasks(ti).id == taskId
                    obj.Tasks(ti).assignedTo = -1;
                    fprintf('[TASK REQUEUE %.2fs] Task %d returned to queue (Robot %d unavailable)\n', ...
                        obj.SimTime, taskId, robotIdx);
                    obj.pushEvent('Task %-3d requeued (R%d down)', taskId, robotIdx);
                    break;
                end
            end
        end

        function order = computeRobotPriorityOrder(obj)
            busy = false(numel(obj.Robots), 1);
            age = zeros(numel(obj.Robots), 1);
            dist = inf(numel(obj.Robots), 1);
            for i = 1:numel(obj.Robots)
                if ~obj.Robots(i).isOperational()
                    continue;
                end
                busy(i) = strcmp(obj.Robots(i).Status, 'busy');
                if busy(i)
                    age(i) = obj.SimTime - obj.getTaskSpawnTime(obj.Robots(i).CurrentTaskID);
                    if ~isnan(obj.Robots(i).TargetX)
                        dist(i) = norm([obj.Robots(i).X - obj.Robots(i).TargetX, ...
                            obj.Robots(i).Y - obj.Robots(i).TargetY]);
                    end
                end
            end
            idx = (1:numel(obj.Robots))';
            score = zeros(size(idx));
            switch obj.PriorityPolicy
                case 'distance_to_goal'
                    score = -dist;
                otherwise
                    score = age;
            end
            ranking = table(idx, busy, score, dist, ...
                'VariableNames', {'idx','busy','score','dist'});
            ranking = sortrows(ranking, {'busy','score','dist'}, {'descend','descend','ascend'});
            order = ranking.idx';
        end

        function updateCriticalZoneReservations(obj, priorityOrder)
            holdTime = 1.2;
            for z = 1:numel(obj.CriticalZones)
                if obj.CriticalZones(z).reservedUntil < obj.SimTime
                    obj.CriticalZones(z).reservedBy = -1;
                end
            end
            for ord = 1:numel(priorityOrder)
                rid = priorityOrder(ord);
                r = obj.Robots(rid);
                if ~r.isOperational() || isnan(r.TargetX)
                    continue;
                end
                for z = 1:numel(obj.CriticalZones)
                    cz = obj.CriticalZones(z);
                    d = norm([r.X - cz.center(1), r.Y - cz.center(2)]);
                    if d > 4.0
                        continue;
                    end
                    claimedByOther = cz.reservedBy > 0 && cz.reservedBy ~= rid && cz.reservedUntil >= obj.SimTime;
                    if claimedByOther
                        continue;
                    end
                    obj.CriticalZones(z).reservedBy = rid;
                    obj.CriticalZones(z).reservedUntil = obj.SimTime + holdTime;
                end
            end
        end

        function enforceZoneAndTaskSpaceProtocols(obj, robotIdx, prioritySlot, priorityOrder)
            r = obj.Robots(robotIdx);
            holdUntil = -Inf;
            boost = 0.0;
            for z = 1:numel(obj.CriticalZones)
                cz = obj.CriticalZones(z);
                d = norm([r.X - cz.center(1), r.Y - cz.center(2)]);
                if d <= (cz.radius + 0.7) && cz.reservedBy > 0 && cz.reservedBy ~= robotIdx
                    holdUntil = max(holdUntil, obj.SimTime + 0.35 + 0.05 * rand());
                end
            end
            if strcmp(r.Status, 'busy') && ~isnan(r.TargetX) && ~isnan(r.TargetY)
                [mustQueue, tokenBoost] = obj.checkTaskSpaceToken(robotIdx);
                boost = max(boost, tokenBoost);
                if mustQueue
                    holdUntil = max(holdUntil, obj.SimTime + 0.35);
                end
            end
            if prioritySlot > 1
                for j = 1:prioritySlot-1
                    h = obj.Robots(priorityOrder(j));
                    if ~h.isOperational()
                        continue;
                    end
                    d = norm([r.X - h.X, r.Y - h.Y]);
                    if d < 1.35
                        holdUntil = max(holdUntil, obj.SimTime + 0.25);
                        break;
                    end
                end
            end
            if isfinite(holdUntil)
                r.setExternalHoldUntil(holdUntil);
            end
            if boost > 0
                r.requestDeadlockRecovery(boost);
                obj.DeadlockRecoveries = obj.DeadlockRecoveries + 1;
                obj.pushEvent('R%d  deadlock recovery boost', robotIdx);
            end
        end

        function [mustQueue, boost] = checkTaskSpaceToken(obj, robotIdx)
            mustQueue = false;
            boost = 0.0;
            r = obj.Robots(robotIdx);
            poiName = obj.findTargetPoiName(r.TargetX, r.TargetY);
            if isempty(poiName)
                return;
            end
            key = matlab.lang.makeValidName(poiName);
            tIdx = find(strcmp({obj.TaskSpaceTokens.name}, key), 1, 'first');
            if isempty(tIdx)
                return;
            end
            token = obj.TaskSpaceTokens(tIdx);
            hasToken = any(token.holders == robotIdx);
            approach = obj.GoalApproachMap.(key);
            dApproach = norm([r.X - approach(1), r.Y - approach(2)]);
            dGoal = norm([r.X - r.TargetX, r.Y - r.TargetY]);
            if hasToken
                if dGoal > 1.6
                    obj.TaskSpaceTokens(tIdx).holders = token.holders(token.holders ~= robotIdx);
                end
                return;
            end
            if dApproach <= 0.9
                if numel(token.holders) < token.capacity
                    obj.TaskSpaceTokens(tIdx).holders(end+1,1) = robotIdx;
                else
                    mustQueue = true;
                    boost = 0.6;
                end
            end
        end

        function name = findTargetPoiName(obj, tx, ty)
            name = '';
            for k = 1:numel(obj.POIs)
                if ~obj.POIs(k).taskTarget
                    continue;
                end
                if norm([obj.POIs(k).x - tx, obj.POIs(k).y - ty]) < 0.35
                    name = obj.POIs(k).name;
                    return;
                end
            end
        end

        function releaseTaskTokenByRobot(obj, robotIdx)
            for t = 1:numel(obj.TaskSpaceTokens)
                holders = obj.TaskSpaceTokens(t).holders;
                obj.TaskSpaceTokens(t).holders = holders(holders ~= robotIdx);
            end
        end

        function detectNearMisses(obj)
            nearPad = 0.25;
            nRobots = numel(obj.Robots);
            for i = 1:nRobots-1
                ri = obj.Robots(i);
                if ~ri.isOperational()
                    continue;
                end
                for j = i+1:nRobots
                    rj = obj.Robots(j);
                    if ~rj.isOperational()
                        continue;
                    end
                    d = norm([ri.X-rj.X, ri.Y-rj.Y]);
                    if d < (ri.Radius + rj.Radius + nearPad) && d >= (ri.Radius + rj.Radius)
                        obj.TotalNearMisses = obj.TotalNearMisses + 1;
                    end
                end
            end
        end

        % ============================================================== %
        % --- Per-layer render helpers (called from render) ----------- %
        % ============================================================== %
        function updateZoneHalos(obj)
            for zi = 1:numel(obj.CriticalZones)
                cz = obj.CriticalZones(zi);
                if zi > numel(obj.hZoneHalos) || ~isvalid(obj.hZoneHalos(zi))
                    continue;
                end
                if ~obj.ShowZones
                    set(obj.hZoneHalos(zi), 'Visible','off');
                    set(obj.hZoneLabels(zi), 'Visible','off');
                    continue;
                end

                if cz.reservedBy > 0 && cz.reservedUntil >= obj.SimTime
                    rid = cz.reservedBy;
                    if rid <= size(obj.RobotIdentityColors,1)
                        col = obj.RobotIdentityColors(rid,:);
                    else
                        col = [0.95 0.55 0.20];
                    end
                    remaining = max(0, cz.reservedUntil - obj.SimTime);
                    set(obj.hZoneHalos(zi), 'EdgeColor', col, ...
                        'FaceColor', col, 'FaceAlpha', 0.18, ...
                        'LineStyle','-', 'LineWidth', 1.4, 'Visible','on');
                    set(obj.hZoneLabels(zi), 'Color', col, 'Visible','on', ...
                        'String', sprintf('%s : R%d (%.1fs)', ...
                            strrep(cz.name,'_','-'), rid, remaining));
                else
                    set(obj.hZoneHalos(zi), 'EdgeColor',[0.30 0.85 0.55], ...
                        'FaceColor',[0.30 0.85 0.55], 'FaceAlpha',0.08, ...
                        'LineStyle','--','LineWidth',1.0,'Visible','on');
                    set(obj.hZoneLabels(zi), 'Color',[0.55 0.78 0.62], ...
                        'Visible','on', ...
                        'String', sprintf('%s : free', strrep(cz.name,'_','-')));
                end
            end
        end

        % ============================================================== %
        function updatePlannedPaths(obj)
            n = numel(obj.Robots);
            % Lazily allocate handles per robot (one polyline each for
            % future and past path segments, plus leader line).
            obj.ensureRobotOverlayHandles(n);
            for i = 1:n
                r = obj.Robots(i);
                col = obj.colorForRobot(i);
                wp = r.PlannedWaypoints;
                if ~obj.ShowPaths || isempty(wp) || ~r.isOperational() || isnan(r.TargetX)
                    set(obj.hPlannedFuture(i), 'Visible','off');
                    set(obj.hPlannedPast(i), 'Visible','off');
                    continue;
                end
                idx = max(1, min(size(wp,1), r.WaypointIndex));
                pastX = [r.X; wp(1:idx,1)];
                pastY = [r.Y; wp(1:idx,2)];
                if idx <= size(wp,1)
                    futX = wp(idx:end,1);
                    futY = wp(idx:end,2);
                else
                    futX = wp(end,1); futY = wp(end,2);
                end
                set(obj.hPlannedFuture(i), 'XData', futX, 'YData', futY, ...
                    'Color', [col 0.95], 'LineWidth', 1.6, ...
                    'LineStyle', '-', 'Visible', 'on');
                pastCol = 0.45 * col + 0.55 * [0.20 0.22 0.30];
                set(obj.hPlannedPast(i), 'XData', pastX, 'YData', pastY, ...
                    'Color', [pastCol 0.55], 'LineWidth', 0.9, ...
                    'LineStyle', ':', 'Visible', 'on');
            end
            for i = n+1:numel(obj.hPlannedFuture)
                if isvalid(obj.hPlannedFuture(i))
                    set(obj.hPlannedFuture(i),'Visible','off');
                end
                if isvalid(obj.hPlannedPast(i))
                    set(obj.hPlannedPast(i),'Visible','off');
                end
            end
        end

        % ============================================================== %
        function updateLeaderLines(obj)
            n = numel(obj.Robots);
            obj.ensureRobotOverlayHandles(n);
            for i = 1:n
                r = obj.Robots(i);
                if ~obj.ShowLeaderLines || ~strcmp(r.Status,'busy') || isnan(r.TargetX)
                    set(obj.hLeaderLines(i),'Visible','off');
                    continue;
                end
                col = obj.colorForRobot(i);
                set(obj.hLeaderLines(i), ...
                    'XData', [r.X r.TargetX], ...
                    'YData', [r.Y r.TargetY], ...
                    'Color', [col 0.40], ...
                    'LineWidth', 0.9, 'LineStyle','--', 'Visible','on');
            end
        end

        % ============================================================== %
        function updateLidar(obj)
            n = numel(obj.Robots);
            % Resize hLidar to n; create missing.
            for i = 1:n
                r = obj.Robots(i);
                showThis = obj.LidarVisible && r.isOperational() && ...
                    (obj.ShowAllLidar || i == obj.FocusedRobotIdx);
                if i > numel(obj.hLidar) || ~isvalid(obj.hLidar(i))
                    obj.hLidar(i) = line(obj.Ax, NaN, NaN, ...
                        'Color', [0.35 0.85 0.35 0.55], ...
                        'LineWidth', 0.40, 'Visible','off');
                end
                if ~showThis || isempty(r.LidarAngles)
                    set(obj.hLidar(i),'Visible','off');
                    continue;
                end
                [xRays, yRays] = obj.packLidarHitSegments(r);
                set(obj.hLidar(i), 'XData', xRays, 'YData', yRays, ...
                    'Visible','on');
            end
            for i = n+1:numel(obj.hLidar)
                if isvalid(obj.hLidar(i))
                    set(obj.hLidar(i),'Visible','off');
                end
            end
        end

        % ============================================================== %
        function updateTrails(obj)
            n = numel(obj.Robots);
            if numel(obj.hTrails) < n
                obj.hTrails(end+1:n) = {[]};
            end
            for i = 1:n
                r = obj.Robots(i);
                if isempty(obj.hTrails{i}) || ~isvalid(obj.hTrails{i})
                    col = obj.colorForRobot(i);
                    obj.hTrails{i} = animatedline(obj.Ax, ...
                        'Color', [col 0.65], 'LineWidth', 1.3, ...
                        'MaximumNumPoints', 240);
                end
                vis = obj.boolToOnOff(obj.ShowTrails);
                set(obj.hTrails{i}, 'Visible', vis);
                if obj.ShowTrails && r.isOperational()
                    addpoints(obj.hTrails{i}, r.X, r.Y);
                end
            end
        end

        % ============================================================== %
        function updateNurses(obj)
            for k = 1:numel(obj.Nurses)
                ne = obj.Nurses(k);
                [bodyX, bodyY, cartX, cartY] = obj.nurseGlyph(ne);

                % Cart
                if ne.HasCart
                    if k > numel(obj.hNurseCarts) || ~isvalid(obj.hNurseCarts(k))
                        obj.hNurseCarts(k) = fill(obj.Ax, cartX, cartY, [0.30 0.66 0.98], ...
                            'EdgeColor',[0.85 0.94 1.0], 'LineWidth',1.0);
                    else
                        set(obj.hNurseCarts(k), 'XData', cartX, 'YData', cartY, 'Visible','on');
                    end
                else
                    if k <= numel(obj.hNurseCarts) && isvalid(obj.hNurseCarts(k))
                        set(obj.hNurseCarts(k), 'Visible','off');
                    end
                end

                % Body
                if k > numel(obj.hNurseBodies) || ~isvalid(obj.hNurseBodies(k))
                    obj.hNurseBodies(k) = fill(obj.Ax, bodyX, bodyY, [0.22 0.78 0.92], ...
                        'EdgeColor',[0.92 1.0 1.0], 'LineWidth',1.2);
                else
                    set(obj.hNurseBodies(k), 'XData', bodyX, 'YData', bodyY);
                end
                if ne.HasCart
                    set(obj.hNurseBodies(k), 'FaceColor', [0.12 0.44 0.84], 'EdgeColor', [0.94 0.98 1.0]);
                else
                    set(obj.hNurseBodies(k), 'FaceColor', [0.22 0.78 0.92], 'EdgeColor', [0.92 1.0 1.0]);
                end

                headingLen = ne.Radius * 1.4;
                hx = [ne.X, ne.X + headingLen*cos(ne.Theta)];
                hy = [ne.Y, ne.Y + headingLen*sin(ne.Theta)];
                if k > numel(obj.hNurseHeadings) || ~isvalid(obj.hNurseHeadings(k))
                    obj.hNurseHeadings(k) = line(obj.Ax, hx, hy, ...
                        'Color',[0.96 0.98 1.0], 'LineWidth',1.0);
                else
                    set(obj.hNurseHeadings(k), 'XData', hx, 'YData', hy, 'Visible','on');
                end

                % ID label
                lbl = sprintf('N%d', k);
                if ne.HasCart, lbl = sprintf('C%d', k); end
                if k > numel(obj.hNurseIdLabels) || ~isvalid(obj.hNurseIdLabels(k))
                    obj.hNurseIdLabels(k) = text(obj.Ax, ne.X, ne.Y - ne.Radius - 0.32, lbl, ...
                        'Color',[0.85 0.92 1.0],'FontSize',7,'FontName','Consolas', ...
                        'HorizontalAlignment','center');
                else
                    set(obj.hNurseIdLabels(k), 'Position', [ne.X, ne.Y - ne.Radius - 0.32, 0], ...
                        'String', lbl, ...
                        'Visible', obj.boolToOnOff(obj.ShowIds));
                end
            end
        end

        % ============================================================== %
        function updateRobots(obj)
            n = numel(obj.Robots);
            obj.ensureRobotOverlayHandles(n);
            for i = 1:n
                r = obj.Robots(i);
                [px, py] = obj.robotGlyph(r);
                idCol = obj.colorForRobot(i);
                bodyCol = obj.statusModulatedColor(idCol, r.Status);
                if strcmp(r.Status, 'out_of_operation')
                    edgeCol = 0.45 * idCol;
                    edgeWidth = 0.8;
                else
                    edgeCol = idCol;
                    edgeWidth = 1.6;
                end
                if i > numel(obj.hRobots) || ~isvalid(obj.hRobots(i))
                    obj.hRobots(i) = fill(obj.Ax, px, py, bodyCol, ...
                        'EdgeColor', edgeCol, 'LineWidth', edgeWidth);
                else
                    set(obj.hRobots(i), 'XData', px, 'YData', py, ...
                        'FaceColor', bodyCol, 'EdgeColor', edgeCol, ...
                        'LineWidth', edgeWidth);
                end

                arrowLen = r.Radius * 1.7;
                hx = [r.X, r.X + arrowLen*cos(r.Theta)];
                hy = [r.Y, r.Y + arrowLen*sin(r.Theta)];
                if i > numel(obj.hRobotHeadings) || ~isvalid(obj.hRobotHeadings(i))
                    obj.hRobotHeadings(i) = line(obj.Ax, hx, hy, ...
                        'Color', [1 1 1], 'LineWidth', 1.6);
                else
                    set(obj.hRobotHeadings(i), 'XData', hx, 'YData', hy, ...
                        'Color', [1 1 1], 'Visible','on');
                end

                lbl = sprintf('R%d', i);
                if i > numel(obj.hRobotIdLabels) || ~isvalid(obj.hRobotIdLabels(i))
                    obj.hRobotIdLabels(i) = text(obj.Ax, r.X, r.Y, lbl, ...
                        'Color', [0.05 0.05 0.08], ...
                        'FontSize', 8, 'FontWeight','bold', ...
                        'FontName','Consolas', ...
                        'HorizontalAlignment','center', ...
                        'VerticalAlignment','middle');
                else
                    set(obj.hRobotIdLabels(i), 'Position',[r.X r.Y 0], ...
                        'String', lbl, ...
                        'Visible', obj.boolToOnOff(obj.ShowIds));
                end

                % Status badge above the robot
                state = r.getDebugState();
                [badgeStr, badgeCol] = obj.badgeFromState(state);
                if i > numel(obj.hRobotStatusBadges) || ~isvalid(obj.hRobotStatusBadges(i))
                    obj.hRobotStatusBadges(i) = text(obj.Ax, r.X, r.Y + r.Radius + 0.55, badgeStr, ...
                        'Color', badgeCol, 'FontSize', 7, 'FontWeight','bold', ...
                        'FontName','Consolas', ...
                        'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
                        'BackgroundColor',[0.05 0.06 0.10], ...
                        'Margin', 1);
                else
                    showBadge = obj.ShowBadges && ~isempty(badgeStr);
                    set(obj.hRobotStatusBadges(i), ...
                        'Position', [r.X, r.Y + r.Radius + 0.55, 0], ...
                        'String', badgeStr, 'Color', badgeCol, ...
                        'Visible', obj.boolToOnOff(showBadge));
                end
            end
        end

        % ============================================================== %
        function updateFocusRing(obj)
            n = numel(obj.Robots);
            if n == 0 || obj.FocusedRobotIdx < 1 || obj.FocusedRobotIdx > n
                if isvalid(obj.hFocusRing)
                    set(obj.hFocusRing,'Visible','off');
                end
                return;
            end
            r = obj.Robots(obj.FocusedRobotIdx);
            th = linspace(0, 2*pi, 48);
            rr = r.Radius * 2.1;
            xc = r.X + rr * cos(th);
            yc = r.Y + rr * sin(th);
            col = obj.colorForRobot(obj.FocusedRobotIdx);
            set(obj.hFocusRing, 'XData', xc, 'YData', yc, ...
                'Color', [col 0.85], 'Visible','on');
        end

        % ============================================================== %
        function updateTasks(obj)
            if isempty(obj.Tasks)
                set(obj.hTasks,'Visible','off');
                return;
            end
            activeMask = [obj.Tasks.assignedTo] == -1;
            if ~any(activeMask)
                set(obj.hTasks,'Visible','off');
                return;
            end
            actX = [obj.Tasks(activeMask).x];
            actY = [obj.Tasks(activeMask).y];
            ages = obj.SimTime - [obj.Tasks(activeMask).spawnTime];
            % Color shift orange (young) → red (older >15 s)
            ageNorm = max(0, min(1, ages / 15));
            % Render single scatter; we approximate average age tint by max
            maxAge = max(ageNorm);
            faceCol = (1-maxAge) * [1.0 0.78 0.30] + maxAge * [1.0 0.32 0.20];
            edgeCol = (1-maxAge) * [1.0 0.45 0.32] + maxAge * [0.95 0.18 0.18];
            set(obj.hTasks, ...
                'XData', actX, 'YData', actY, ...
                'MarkerFaceColor', faceCol, ...
                'MarkerEdgeColor', edgeCol, ...
                'Visible','on');
        end

        % ============================================================== %
        function updateTaskFx(obj)
            if isempty(obj.TaskFx)
                return;
            end
            keep = false(numel(obj.TaskFx),1);
            for k = 1:numel(obj.TaskFx)
                fx = obj.TaskFx(k);
                age = obj.SimTime - fx.spawnTime;
                if age > 0.6 || ~isvalid(fx.hPulse)
                    if isvalid(fx.hPulse)
                        delete(fx.hPulse);
                    end
                    continue;
                end
                t = age / 0.6;
                r = 0.6 + 1.4 * t;
                alpha = max(0, 1 - t);
                th = linspace(0,2*pi,32);
                tIdx = arrayfun(@(z)z.id, obj.Tasks) == fx.id;
                if ~any(tIdx)
                    delete(fx.hPulse);
                    continue;
                end
                tk = obj.Tasks(find(tIdx,1,'first'));
                xc = tk.x + r*cos(th);
                yc = tk.y + r*sin(th);
                set(fx.hPulse, 'XData', xc, 'YData', yc, ...
                    'Color', [1.0 0.78 0.30 alpha]);
                keep(k) = true;
            end
            obj.TaskFx = obj.TaskFx(keep);
        end

        % ============================================================== %
        function updateCollisionFx(obj)
            if isempty(obj.CollisionFx)
                return;
            end
            keep = false(numel(obj.CollisionFx),1);
            for k = 1:numel(obj.CollisionFx)
                fx = obj.CollisionFx(k);
                age = obj.SimTime - fx.spawnTime;
                if age > 0.8
                    if isvalid(fx.hRing), delete(fx.hRing); end
                    if isvalid(fx.hText), delete(fx.hText); end
                    continue;
                end
                t = age / 0.8;
                r = 0.5 + 2.5 * t;
                alpha = max(0, 1 - t);
                th = linspace(0,2*pi,32);
                xc = fx.x + r*cos(th);
                yc = fx.y + r*sin(th);
                if isvalid(fx.hRing)
                    set(fx.hRing,'XData',xc,'YData',yc, ...
                        'Color',[1.0 0.18 0.18 alpha]);
                end
                if isvalid(fx.hText)
                    set(fx.hText, ...
                        'Position',[fx.x fx.y+0.3 0], ...
                        'Color',[1.0 0.30 0.30]);
                end
                keep(k) = true;
            end
            obj.CollisionFx = obj.CollisionFx(keep);
        end

        % ============================================================== %
        function ensureRobotOverlayHandles(obj, n)
            % Lazily allocate per-robot overlay handles up to n.
            for i = numel(obj.hPlannedFuture)+1:n
                obj.hPlannedFuture(i) = line(obj.Ax, NaN, NaN, ...
                    'LineWidth',1.6,'Visible','off');
            end
            for i = numel(obj.hPlannedPast)+1:n
                obj.hPlannedPast(i) = line(obj.Ax, NaN, NaN, ...
                    'LineStyle',':','LineWidth',0.9,'Visible','off');
            end
            for i = numel(obj.hLeaderLines)+1:n
                obj.hLeaderLines(i) = line(obj.Ax, NaN, NaN, ...
                    'LineStyle','--','LineWidth',0.9,'Visible','off');
            end
        end

        % ============================================================== %
        function col = colorForRobot(obj, i)
            if i <= size(obj.RobotIdentityColors,1)
                col = obj.RobotIdentityColors(i,:);
            else
                col = [0.80 0.80 0.85];
            end
        end

        % ============================================================== %
        function bodyCol = statusModulatedColor(~, idCol, status)
            switch status
                case 'busy'
                    bodyCol = idCol;
                case 'idle'
                    bodyCol = 0.55 * idCol + 0.45 * [0.20 0.22 0.30];
                case 'error'
                    bodyCol = [0.96 0.24 0.24];
                case 'out_of_operation'
                    bodyCol = [0.30 0.30 0.34];
                otherwise
                    bodyCol = idCol;
            end
        end

        % ============================================================== %
        function [str, col] = badgeFromState(~, state)
            switch state
                case 'cruising', str = '';        col = [0.80 0.92 1.00];
                case 'idle',     str = 'IDLE';    col = [0.78 0.84 0.92];
                case 'yielding', str = 'YIELD';   col = [0.95 0.85 0.30];
                case 'holding',  str = 'HOLD';    col = [0.55 0.78 1.00];
                case 'reversing',str = 'REV';     col = [0.95 0.55 0.20];
                case 'boosted',  str = 'BOOST';   col = [0.95 0.45 0.85];
                case 'down',     str = 'DOWN';    col = [0.95 0.30 0.30];
                otherwise,       str = '';        col = [0.80 0.80 0.80];
            end
        end

        % ============================================================== %
        function s = layerInfoString(obj)
            s = sprintf(['  Layers  L:lidar(%s)  A:all-lidar(%s)  P:paths(%s)  G:goal(%s)  Z:zones(%s)\n', ...
                         '          T:trails(%s)  B:badges(%s)  I:ids(%s)  TAB:focus R%d'], ...
                obj.boolStr(obj.LidarVisible), obj.boolStr(obj.ShowAllLidar), ...
                obj.boolStr(obj.ShowPaths), obj.boolStr(obj.ShowLeaderLines), ...
                obj.boolStr(obj.ShowZones), obj.boolStr(obj.ShowTrails), ...
                obj.boolStr(obj.ShowBadges), obj.boolStr(obj.ShowIds), ...
                obj.FocusedRobotIdx);
        end

        function s = boolStr(~, tf)
            if tf, s = 'on'; else, s = 'off'; end
        end

        function toggleLidarFromButton(obj)
            obj.LidarVisible = ~obj.LidarVisible;
            obj.updateLidar();
        end

        % ============================================================== %
        function s = kpiTextString(obj)
            avgTime = 0;
            if ~isempty(obj.CompletionTimes)
                avgTime = mean(obj.CompletionTimes);
            end
            nDone = numel(obj.CompletionTimes);
            nQueue = 0;
            if ~isempty(obj.Tasks)
                nQueue = sum([obj.Tasks.assignedTo] == -1);
            end
            s = sprintf([ ...
                '  Collisions          : %d\n', ...
                '  Near misses         : %d\n', ...
                '  Deadlock recoveries : %d\n', ...
                '  Tasks completed     : %d\n', ...
                '  Avg task time       : %.1f s\n', ...
                '  Active queue        : %d'], ...
                obj.TotalCollisions, obj.TotalNearMisses, ...
                obj.DeadlockRecoveries, nDone, avgTime, nQueue);
        end

        % ============================================================== %
        function s = legendTextString(~)
            s = [ ...
                '  rect      Robot (color = identity)' newline ...
                '  -- line   Planned path (future)' newline ...
                '  .. line   Path traveled' newline ...
                '  ring      Critical zone (green=free)' newline ...
                '  pent      Pending task (red=aged)' newline ...
                '  flash     Collision event' newline ...
                '  YIELD     Holding for nurse' newline ...
                '  HOLD      Zone/token reserved' newline ...
                '  REV       Reversing (deadlock)' newline ...
                '  BOOST     Deadlock recovery boost'];
        end

        % ============================================================== %
        function updateDashboard(obj)
            if ~isvalid(obj.hKpiText), return; end
            obj.hKpiText.String = obj.kpiTextString();

            % Throughput sparkline (rolling 30 s window)
            window = 30.0;
            if ~isempty(obj.ThroughputHistory)
                t0 = max(0, obj.SimTime - window);
                hist = obj.ThroughputHistory;
                hist(end+1,:) = [obj.SimTime, numel(obj.CompletionTimes)];
                mask = hist(:,1) >= t0;
                xs = hist(mask,1);
                ys = hist(mask,2);
                if ~isempty(xs)
                    set(obj.hThroughputLine, 'XData', xs, 'YData', ys);
                    obj.hThroughputAx.XLim = [max(0,t0), max(obj.SimTime, t0+1)];
                    obj.hThroughputAx.YLim = [0, max(1, max(ys)+1)];
                end
            end

            % Robot status cards
            obj.updateRobotCards();

            % Active task queue
            qLines = {};
            if ~isempty(obj.Tasks)
                queued = obj.Tasks([obj.Tasks.assignedTo] == -1);
                for k = 1:min(6, numel(queued))
                    age = obj.SimTime - queued(k).spawnTime;
                    qLines{end+1} = sprintf('  T%-3d  %-18s  %4.1fs', ...
                        queued(k).id, ...
                        obj.truncString(queued(k).poiName, 18), age); %#ok<AGROW>
                end
                if numel(queued) > 6
                    qLines{end+1} = sprintf('  ... +%d more', numel(queued)-6);
                end
            end
            if isempty(qLines)
                obj.hQueueText.String = '  (empty)';
            else
                obj.hQueueText.String = strjoin(qLines, newline);
            end

            % Event log
            if isempty(obj.EventBuffer)
                obj.hEventLogText.String = '  (no events yet)';
            else
                lastN = obj.EventBuffer(max(1,end-obj.MaxEventLogLines+1):end);
                obj.hEventLogText.String = strjoin(lastN, newline);
            end
        end

        function updateRobotCards(obj)
            n = numel(obj.Robots);
            % Card layout: stack 5 cards from y=0.55 down to y=0.36.
            top = 0.555;
            cardH = 0.030;
            gap = 0.005;
            for i = 1:n
                yTop = top - (i-1) * (cardH + gap);
                yBot = yTop - cardH;
                xL = 0.04; xR = 0.96;
                if i > numel(obj.hRobotCardPatches) || ~isvalid(obj.hRobotCardPatches(i))
                    obj.hRobotCardPatches(i) = patch(obj.hDashAx, ...
                        [xL xR xR xL xL], [yBot yBot yTop yTop yBot], ...
                        [0.10 0.12 0.18], 'EdgeColor',[0.18 0.22 0.32]);
                    obj.hRobotCardStripes(i) = patch(obj.hDashAx, ...
                        [xL xL+0.012 xL+0.012 xL xL], ...
                        [yBot yBot yTop yTop yBot], ...
                        [0.50 0.50 0.50], 'EdgeColor','none');
                    obj.hRobotCardTexts(i) = text(obj.hDashAx, xL+0.02, (yTop+yBot)/2, '', ...
                        'Color',[0.92 0.96 1.00], 'FontName','Consolas','FontSize',9, ...
                        'VerticalAlignment','middle');
                end
                r = obj.Robots(i);
                col = obj.colorForRobot(i);
                state = r.getDebugState();
                if i == obj.FocusedRobotIdx
                    set(obj.hRobotCardPatches(i),'EdgeColor',col,'LineWidth',1.6);
                else
                    set(obj.hRobotCardPatches(i),'EdgeColor',[0.18 0.22 0.32],'LineWidth',0.6);
                end
                if isvalid(obj.hRobotCardStripes(i))
                    set(obj.hRobotCardStripes(i), 'FaceColor', col);
                end
                if r.isOperational()
                    if strcmp(r.Status,'busy') && ~isnan(r.TargetX)
                        d = norm([r.X-r.TargetX, r.Y-r.TargetY]);
                        info = sprintf('  R%d  %-9s  d=%.1fm  T%d', i, state, d, r.CurrentTaskID);
                    else
                        info = sprintf('  R%d  %-9s', i, state);
                    end
                else
                    info = sprintf('  R%d  DOWN', i);
                end
                set(obj.hRobotCardTexts(i),'String', info, ...
                    'Position',[xL+0.02, (yTop+yBot)/2, 0]);
            end
        end

        function s = truncString(~, str, n)
            if isempty(str), s = ''; return; end
            if numel(str) > n
                s = [str(1:max(1,n-1)) '~'];
            else
                s = str;
            end
        end

        % ============================================================== %
        function pushEvent(obj, fmt, varargin)
            line_ = sprintf(['[%5.1fs] ' fmt], obj.SimTime, varargin{:});
            obj.EventBuffer{end+1} = line_;
            if numel(obj.EventBuffer) > 4 * obj.MaxEventLogLines
                obj.EventBuffer = obj.EventBuffer(end-2*obj.MaxEventLogLines+1:end);
            end
        end

        function recordTaskSpawn(obj, taskId, x, y)
            if ~obj.RenderingEnabled
                return;
            end
            th = linspace(0,2*pi,32);
            r = 0.6;
            xc = x + r*cos(th); yc = y + r*sin(th);
            h = line(obj.Ax, xc, yc, 'Color',[1.0 0.78 0.30 1.0], 'LineWidth',1.6);
            obj.TaskFx(end+1) = struct('id',taskId,'spawnTime',obj.SimTime,'hPulse',h);
        end

        function recordCollisionFx(obj, x, y)
            if ~obj.RenderingEnabled
                return;
            end
            th = linspace(0,2*pi,32);
            r = 0.5;
            xc = x + r*cos(th); yc = y + r*sin(th);
            hRing = line(obj.Ax, xc, yc, 'Color',[1.0 0.20 0.20 1.0], 'LineWidth',2.0);
            hText = text(obj.Ax, x, y+0.3, '!', ...
                'Color',[1.0 0.30 0.30],'FontSize',16,'FontWeight','bold', ...
                'HorizontalAlignment','center');
            obj.CollisionFx(end+1) = struct('x',x,'y',y, ...
                'spawnTime',obj.SimTime,'hRing',hRing,'hText',hText);
        end

        function recordCompletion(obj)
            obj.ThroughputHistory(end+1,:) = [obj.SimTime, numel(obj.CompletionTimes)];
        end
    end

    methods (Access = private, Static)
        function [x, y] = roundedRectLocal(len, wid, r, nArc)
            xMin = -len / 2; xMax = len / 2;
            yMin = -wid / 2; yMax = wid / 2;
            r = max(1e-4, min([r, len/2 - 1e-4, wid/2 - 1e-4]));
            % Build a single continuous CCW contour to avoid self-intersections.
            x = [xMax-r, xMin+r];
            y = [yMax,   yMax];
            ang = linspace(pi/2, pi, nArc);
            x = [x, (xMin+r) + r*cos(ang)]; %#ok<AGROW>
            y = [y, (yMax-r) + r*sin(ang)]; %#ok<AGROW>
            x = [x, xMin, xMin]; %#ok<AGROW>
            y = [y, yMin+r, yMin+r]; %#ok<AGROW>
            ang = linspace(pi, 3*pi/2, nArc);
            x = [x, (xMin+r) + r*cos(ang)]; %#ok<AGROW>
            y = [y, (yMin+r) + r*sin(ang)]; %#ok<AGROW>
            x = [x, xMin+r, xMax-r]; %#ok<AGROW>
            y = [y, yMin,   yMin]; %#ok<AGROW>
            ang = linspace(3*pi/2, 2*pi, nArc);
            x = [x, (xMax-r) + r*cos(ang)]; %#ok<AGROW>
            y = [y, (yMin+r) + r*sin(ang)]; %#ok<AGROW>
            x = [x, xMax, xMax]; %#ok<AGROW>
            y = [y, yMin+r, yMax-r]; %#ok<AGROW>
            ang = linspace(0, pi/2, nArc);
            x = [x, (xMax-r) + r*cos(ang)]; %#ok<AGROW>
            y = [y, (yMax-r) + r*sin(ang)]; %#ok<AGROW>
            x(end+1) = x(1);
            y(end+1) = y(1);
        end
    end
end
