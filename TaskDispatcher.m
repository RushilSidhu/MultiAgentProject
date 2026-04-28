classdef TaskDispatcher < handle
    % TaskDispatcher  Generates patient-tending requests for the simulation.
    %
    %   Tasks arrive at an average rate of 1 per 5 seconds with inter-arrival
    %   times drawn from a normal distribution (truncated positive).
    %   Occasional batch arrivals (2-3 tasks) are also supported.
    %
    %   Usage
    %   -----
    %     td = TaskDispatcher(env, meanInterval, batchProb);
    %     td.update(dt);   % call every simulation step

    % ------------------------------------------------------------------ %
    properties (SetAccess = private)
        Env             HospitalEnv     % reference to environment
        MeanInterval    (1,1) double    % [s] average inter-arrival time
        StdInterval     (1,1) double    % [s] std dev of inter-arrival time
        BatchProb       (1,1) double    % probability that an arrival is a batch
        MaxBatchSize    (1,1) double = 3

        TimeUntilNext   (1,1) double    % [s] countdown to next spawn
        TotalSpawned    (1,1) double = 0
    end

    % ------------------------------------------------------------------ %
    methods
        % ============================================================== %
        function obj = TaskDispatcher(env, meanInterval, batchProb)
            obj.Env          = env;
            obj.MeanInterval = 5.0;   % default: 1 task every 5 s
            obj.StdInterval  = 1.5;
            obj.BatchProb    = 0.10;  % 10% chance of a batch

            if nargin >= 2, obj.MeanInterval = meanInterval; end
            if nargin >= 3, obj.BatchProb    = batchProb;    end

            % Sample the first arrival time
            obj.TimeUntilNext = obj.sampleInterval();
        end

        % ============================================================== %
        function update(obj, dt)
            % Call once per simulation time step.
            obj.TimeUntilNext = obj.TimeUntilNext - dt;
            if obj.TimeUntilNext <= 0
                % Determine how many tasks to spawn this event
                if rand() < obj.BatchProb
                    n = randi([2, obj.MaxBatchSize]);
                else
                    n = 1;
                end
                for i = 1:n
                    obj.Env.spawnTask();
                    obj.TotalSpawned = obj.TotalSpawned + 1;
                end
                obj.TimeUntilNext = obj.sampleInterval();
            end
        end
    end

    % ------------------------------------------------------------------ %
    methods (Access = private)
        function t = sampleInterval(obj)
            % Truncated-normal: resample until positive
            while true
                t = obj.MeanInterval + obj.StdInterval * randn();
                if t > 0.5, return; end   % minimum 0.5 s gap
            end
        end
    end
end
