import { useEffect, useMemo, useRef, useState } from 'react';
import { CanvasRenderer, type CanvasLayerSettings } from './render/CanvasRenderer';
import { hospitalScenario } from './scenarios/hospitalScenario';
import { SimulationEngine, type SimulationConfig } from './sim/engine';
import { createRobotPolicy, robotPolicyOptions, type RobotPolicyId } from './sim/policies';
import type { SimulationSnapshot } from './sim/types';
import './styles.css';

const defaultConfig: SimulationConfig = {
  seed: 11,
  robotCount: 4,
  staffCount: 2,
  cartStaffCount: 0,
  taskMeanInterval: 5,
  dt: 0.1,
};

const maxReplayFrames = 900;
const defaultPolicyId: RobotPolicyId = 'cbf';
const defaultLayers: CanvasLayerSettings = {
  grid: true,
  trails: true,
  sensing: true,
  labels: true,
  routes: true,
  doors: true,
};

type TrialResult = {
  seed: number;
  score: number;
  taskCost: number;
  humanMultiplier: number;
  completedTasks: number;
  averageCompletionSeconds: number;
  collisions: number;
  collisionRobotStaff: number;
  collisionRobotRobot: number;
  collisionRobotWall: number;
  nearMisses: number;
  robotUtilization: number;
};

export default function App() {
  const [config, setConfig] = useState(defaultConfig);
  const [policyId, setPolicyId] = useState<RobotPolicyId>(defaultPolicyId);
  const [running, setRunning] = useState(true);
  const [speed, setSpeed] = useState(1);
  const [replayIndex, setReplayIndex] = useState<number | null>(null);
  const [selectedRobotId, setSelectedRobotId] = useState('R1');
  const [layers, setLayers] = useState<CanvasLayerSettings>(defaultLayers);
  const [trialCount, setTrialCount] = useState(10);
  const [trialDuration, setTrialDuration] = useState(120);
  const [trialResults, setTrialResults] = useState<TrialResult[]>([]);
  const [trialProgress, setTrialProgress] = useState<{ current: number; total: number } | null>(null);
  const trialAbortRef = useRef(false);
  const engineRef = useRef(new SimulationEngine(hospitalScenario, createRobotPolicy(defaultPolicyId), config));
  const [snapshot, setSnapshot] = useState<SimulationSnapshot>(() => engineRef.current.snapshot());
  const [history, setHistory] = useState<SimulationSnapshot[]>([snapshot]);
  const selectedSnapshot = replayIndex === null ? snapshot : history[replayIndex] ?? snapshot;
  const selectedRobot = selectedSnapshot.robots.find((robot) => robot.id === selectedRobotId) ?? selectedSnapshot.robots[0];
  const selectedTask = selectedRobot?.taskId
    ? selectedSnapshot.tasks.find((task) => task.id === selectedRobot.taskId)
    : undefined;
  const selectedPolicy = robotPolicyOptions.find((policy) => policy.id === policyId) ?? robotPolicyOptions[0];

  const recentEvents = useMemo(() => selectedSnapshot.events.slice(-8).reverse(), [selectedSnapshot.events]);
  const story = useMemo(() => describeStory(selectedSnapshot, selectedRobot?.id), [selectedSnapshot, selectedRobot?.id]);

  useEffect(() => {
    if (!running || replayIndex !== null) return;
    let frame = 0;
    let raf = 0;
    const tick = () => {
      frame += 1;
      const steps = Math.max(1, Math.round(speed));
      let next = engineRef.current.snapshot();
      for (let i = 0; i < steps; i += 1) {
        next = engineRef.current.step(config.dt);
      }
      setSnapshot(next);
      if (frame % 2 === 0) {
        setHistory((items) => [...items.slice(-(maxReplayFrames - 1)), next]);
      }
      raf = window.setTimeout(tick, 1000 / 30) as unknown as number;
    };
    raf = window.setTimeout(tick, 1000 / 30) as unknown as number;
    return () => window.clearTimeout(raf);
  }, [config.dt, running, replayIndex, speed]);

  const reset = (nextConfig = config, nextPolicyId = policyId) => {
    engineRef.current = new SimulationEngine(hospitalScenario, createRobotPolicy(nextPolicyId), nextConfig);
    const initial = engineRef.current.snapshot();
    setSnapshot(initial);
    setHistory([initial]);
    setReplayIndex(null);
  };

  const stepOnce = () => {
    setRunning(false);
    setReplayIndex(null);
    const next = engineRef.current.step(config.dt);
    setSnapshot(next);
    setHistory((items) => [...items.slice(-(maxReplayFrames - 1)), next]);
  };

  const updateSeed = (value: number) => {
    const next = { ...config, seed: value };
    setConfig(next);
    reset(next, policyId);
  };

  const updateConfig = (patch: Partial<SimulationConfig>) => {
    const next = { ...config, ...patch };
    setConfig(next);
    reset(next, policyId);
  };

  const updatePolicy = (nextPolicyId: RobotPolicyId) => {
    setPolicyId(nextPolicyId);
    reset(config, nextPolicyId);
  };

  const toggleLayer = (key: keyof CanvasLayerSettings) => {
    setLayers((current) => ({ ...current, [key]: !current[key] }));
  };

  const runTrials = async () => {
    if (trialProgress) return;
    setRunning(false);
    setTrialResults([]);
    trialAbortRef.current = false;
    setTrialProgress({ current: 0, total: trialCount });

    const collected: TrialResult[] = [];
    for (let i = 0; i < trialCount; i += 1) {
      if (trialAbortRef.current) break;
      const trialConfig: SimulationConfig = { ...config, seed: config.seed + i };
      const engine = new SimulationEngine(hospitalScenario, createRobotPolicy(policyId), trialConfig);
      const final = engine.runFor(trialDuration);
      const m = final.metrics;
      collected.push({
        seed: trialConfig.seed,
        score: m.score.score,
        taskCost: m.score.taskCost,
        humanMultiplier: m.score.humanMultiplier,
        completedTasks: m.completedTasks,
        averageCompletionSeconds: m.averageCompletionSeconds,
        collisions: m.collisions,
        collisionRobotStaff: m.collisionBreakdown.robotStaff,
        collisionRobotRobot: m.collisionBreakdown.robotRobot,
        collisionRobotWall: m.collisionBreakdown.robotWall,
        nearMisses: m.nearMisses,
        robotUtilization: m.robotUtilization,
      });
      setTrialProgress({ current: i + 1, total: trialCount });
      setTrialResults([...collected]);
      // Yield to the UI so the progress indicator updates between trials
      await new Promise((resolve) => setTimeout(resolve, 0));
    }
    setTrialProgress(null);
  };

  const cancelTrials = () => {
    trialAbortRef.current = true;
  };

  return (
    <main className="app-shell">
      <section className="hero">
        <div>
          <p className="eyebrow">Visual Multi-Agent Simulation Revamp</p>
          <h1>Hospital Delivery Fleet Control Room</h1>
          <p>
            A deterministic browser simulation with explicit scenario data, event telemetry,
            staff traffic, task queues, and a swappable robot policy API.
          </p>
        </div>
        <div className="hero-card">
          <strong>{selectedPolicy.label}</strong>
          <span>Policy boundary: observations in, validated actions out.</span>
        </div>
      </section>

      <section className="sim-layout">
        <div className="viewport-card">
          <CanvasRenderer
            snapshot={selectedSnapshot}
            selectedTime={replayIndex === null ? undefined : selectedSnapshot.time}
            selectedRobotId={selectedRobot?.id}
            layers={layers}
            onSelectRobot={setSelectedRobotId}
          />
        </div>

        <aside className="side-panel">
          <div className="panel-card story">
            <h2>Presentation Narrative</h2>
            <p>{story}</p>
            {selectedRobot?.diagnostics && (
              <div className="robot-focus">
                <strong>{selectedRobot.id}: {selectedRobot.diagnostics.phase.toUpperCase()}</strong>
                <span>{selectedRobot.diagnostics.note}</span>
                {selectedRobot.diagnostics.targetLabel && <span>Target: {selectedRobot.diagnostics.targetLabel}</span>}
                {selectedTask && <span>Task: {selectedTask.label} · {selectedTask.status}</span>}
              </div>
            )}
          </div>

          <div className="panel-card controls">
            <h2>Run Controls</h2>
            <div className="button-row">
              <button onClick={() => setRunning((value) => !value)}>{running ? 'Pause' : 'Run'}</button>
              <button onClick={stepOnce}>Step</button>
              <button onClick={() => reset()}>Reset</button>
            </div>
            <label>
              Seed
              <input
                type="number"
                value={config.seed}
                onChange={(event) => updateSeed(Number(event.target.value) || 1)}
              />
            </label>
            <label>
              Robot Strategy
              <select value={policyId} onChange={(event) => updatePolicy(event.target.value as RobotPolicyId)}>
                {robotPolicyOptions.map((policy) => (
                  <option value={policy.id} key={policy.id}>
                    {policy.label}
                  </option>
                ))}
              </select>
            </label>
            <label>
              Speed {speed.toFixed(0)}x
              <input
                type="range"
                min="1"
                max="8"
                value={speed}
                onChange={(event) => setSpeed(Number(event.target.value))}
              />
            </label>
            <div className="config-grid">
              <label>
                Robots
                <input
                  type="number"
                  min="1"
                  max="8"
                  value={config.robotCount}
                  onChange={(event) => updateConfig({ robotCount: clampInteger(event.target.value, 1, 8) })}
                />
              </label>
              <label>
                Staff
                <input
                  type="number"
                  min="0"
                  max="12"
                  value={config.staffCount}
                  onChange={(event) => updateConfig({ staffCount: clampInteger(event.target.value, 0, 12) })}
                />
              </label>
              <label>
                Carts
                <input
                  type="number"
                  min="0"
                  max="8"
                  value={config.cartStaffCount}
                  onChange={(event) => updateConfig({ cartStaffCount: clampInteger(event.target.value, 0, 8) })}
                />
              </label>
              <label>
                Task Gap
                <input
                  type="number"
                  min="1"
                  max="20"
                  value={config.taskMeanInterval}
                  onChange={(event) => updateConfig({ taskMeanInterval: clampInteger(event.target.value, 1, 20) })}
                />
              </label>
            </div>
            <label>
              Replay {history.length === 0 ? '0 / 0' : `${(replayIndex ?? history.length - 1) + 1} / ${history.length}`}
              <input
                type="range"
                min="0"
                max={Math.max(0, history.length - 1)}
                value={replayIndex ?? history.length - 1}
                onChange={(event) => {
                  setRunning(false);
                  setReplayIndex(Number(event.target.value));
                }}
              />
            </label>
            {replayIndex !== null && <button onClick={() => setReplayIndex(null)}>Return To Live</button>}
          </div>

          <div className="panel-card layers">
            <h2>Visual Layers</h2>
            <div className="toggle-grid">
              {Object.entries(layers).map(([key, enabled]) => (
                <button
                  className={enabled ? 'toggle active' : 'toggle'}
                  key={key}
                  onClick={() => toggleLayer(key as keyof CanvasLayerSettings)}
                >
                  {key}
                </button>
              ))}
            </div>
          </div>

          <div className="panel-card robot-selector">
            <h2>Robot Focus</h2>
            <div className="robot-button-grid">
              {selectedSnapshot.robots.map((robot) => (
                <button
                  className={robot.id === selectedRobot?.id ? 'robot-button active' : 'robot-button'}
                  key={robot.id}
                  onClick={() => setSelectedRobotId(robot.id)}
                >
                  <strong>{robot.id}</strong>
                  <span>{robot.status}</span>
                </button>
              ))}
            </div>
          </div>

          <div className="panel-card metrics">
            <h2>Live Metrics</h2>
            <Metric label="Score S" value={selectedSnapshot.metrics.score.score.toFixed(3)} />
            <Metric label="C_task" value={selectedSnapshot.metrics.score.taskCost.toFixed(3)} />
            <Metric label="M_human" value={`${selectedSnapshot.metrics.score.humanMultiplier.toFixed(1)}x`} />
            <Metric label="Completed" value={selectedSnapshot.metrics.completedTasks.toString()} />
            <Metric label="Active Tasks" value={selectedSnapshot.metrics.activeTasks.toString()} />
            <Metric label="Queued" value={selectedSnapshot.metrics.queuedTasks.toString()} />
            <Metric label="Collisions" value={selectedSnapshot.metrics.collisions.toString()} />
            <Metric label="Robot-Staff" value={selectedSnapshot.metrics.collisionBreakdown.robotStaff.toString()} />
            <Metric label="Robot-Robot" value={selectedSnapshot.metrics.collisionBreakdown.robotRobot.toString()} />
            <Metric label="Robot-Wall" value={selectedSnapshot.metrics.collisionBreakdown.robotWall.toString()} />
            <Metric label="Near Miss Episodes" value={selectedSnapshot.metrics.nearMisses.toString()} />
            <Metric label="Avg Completion" value={`${selectedSnapshot.metrics.averageCompletionSeconds.toFixed(1)}s`} />
            <Metric label="Robot Utilization" value={`${(selectedSnapshot.metrics.robotUtilization * 100).toFixed(0)}%`} />
          </div>

          <div className="panel-card trial-sweep">
            <h2>Trial Sweep</h2>
            <p className="muted">
              Runs N independent trials at seeds {config.seed}…{config.seed + trialCount - 1} with the
              current policy and agent counts, then averages the metrics.
            </p>
            <div className="config-grid">
              <label>
                Trials
                <input
                  type="number"
                  min="1"
                  max="200"
                  value={trialCount}
                  onChange={(event) => setTrialCount(clampInteger(event.target.value, 1, 200))}
                  disabled={trialProgress !== null}
                />
              </label>
              <label>
                Duration (s)
                <input
                  type="number"
                  min="10"
                  max="1800"
                  value={trialDuration}
                  onChange={(event) => setTrialDuration(clampInteger(event.target.value, 10, 1800))}
                  disabled={trialProgress !== null}
                />
              </label>
            </div>
            <div className="button-row">
              <button onClick={runTrials} disabled={trialProgress !== null}>
                {trialProgress ? 'Running…' : 'Run Sweep'}
              </button>
              {trialProgress && <button onClick={cancelTrials}>Cancel</button>}
            </div>
            {trialProgress && (
              <p>
                Progress: {trialProgress.current} / {trialProgress.total}
              </p>
            )}
            {trialResults.length > 0 && <TrialAggregate results={trialResults} />}
          </div>

          <div className="panel-card queue">
            <h2>Task Queue</h2>
            {selectedSnapshot.tasks.filter((task) => task.status !== 'completed').slice(0, 7).map((task) => (
              <div className="task-card" key={task.id}>
                <strong>{task.id} · {task.label}</strong>
                <span>{task.status} · P{task.priority} · due {Math.max(0, task.deadlineAt - selectedSnapshot.time).toFixed(0)}s</span>
              </div>
            ))}
            {selectedSnapshot.tasks.filter((task) => task.status !== 'completed').length === 0 && (
              <p className="muted">No active requests yet.</p>
            )}
          </div>

          <div className="panel-card events">
            <h2>Event Feed</h2>
            {recentEvents.map((event, index) => (
              <p key={`${event.time}-${index}`}>
                <span>{event.time.toFixed(1)}s</span> {describeEvent(event)}
              </p>
            ))}
          </div>
        </aside>
      </section>
    </main>
  );
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="metric-row">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function meanStd(values: number[]): { mean: number; std: number } {
  if (values.length === 0) return { mean: 0, std: 0 };
  const mean = values.reduce((sum, v) => sum + v, 0) / values.length;
  if (values.length === 1) return { mean, std: 0 };
  const variance =
    values.reduce((sum, v) => sum + (v - mean) * (v - mean), 0) / (values.length - 1);
  return { mean, std: Math.sqrt(variance) };
}

function TrialAggregate({ results }: { results: TrialResult[] }) {
  const fields: Array<{ label: string; pick: (r: TrialResult) => number; format: (v: number) => string }> = [
    { label: 'Score S', pick: (r) => r.score, format: (v) => v.toFixed(3) },
    { label: 'C_task', pick: (r) => r.taskCost, format: (v) => v.toFixed(3) },
    { label: 'M_human', pick: (r) => r.humanMultiplier, format: (v) => `${v.toFixed(2)}x` },
    { label: 'Completed', pick: (r) => r.completedTasks, format: (v) => v.toFixed(1) },
    { label: 'Avg Completion (s)', pick: (r) => r.averageCompletionSeconds, format: (v) => v.toFixed(1) },
    { label: 'Collisions', pick: (r) => r.collisions, format: (v) => v.toFixed(1) },
    { label: 'Robot-Staff', pick: (r) => r.collisionRobotStaff, format: (v) => v.toFixed(1) },
    { label: 'Robot-Robot', pick: (r) => r.collisionRobotRobot, format: (v) => v.toFixed(1) },
    { label: 'Robot-Wall', pick: (r) => r.collisionRobotWall, format: (v) => v.toFixed(1) },
    { label: 'Near Misses', pick: (r) => r.nearMisses, format: (v) => v.toFixed(1) },
    { label: 'Utilization', pick: (r) => r.robotUtilization, format: (v) => `${(v * 100).toFixed(0)}%` },
  ];
  return (
    <div className="trial-aggregate">
      <h3>Averages over {results.length} trial{results.length === 1 ? '' : 's'}</h3>
      {fields.map((field) => {
        const stats = meanStd(results.map(field.pick));
        return (
          <div className="metric-row" key={field.label}>
            <span>{field.label}</span>
            <strong>
              {field.format(stats.mean)}
              {results.length > 1 && <span className="muted"> ± {field.format(stats.std)}</span>}
            </strong>
          </div>
        );
      })}
    </div>
  );
}

function describeEvent(event: SimulationSnapshot['events'][number]): string {
  switch (event.kind) {
    case 'task-created':
      return `created ${event.taskId} (${event.label})`;
    case 'task-assigned':
      return `assigned ${event.taskId} to ${event.robotId}`;
    case 'task-completed':
      return `${event.robotId} completed ${event.taskId} in ${event.elapsed.toFixed(1)}s`;
    case 'collision':
      return `${event.severity}: ${event.a} vs ${event.b}`;
    case 'robot-blocked':
      return `${event.robotId} blocked: ${event.reason}`;
  }
}

function describeStory(snapshot: SimulationSnapshot, selectedRobotId?: string): string {
  const activeTasks = snapshot.tasks.filter((task) => task.status !== 'completed' && task.status !== 'cancelled');
  const blockedRobots = snapshot.robots.filter((robot) => robot.status === 'blocked');
  const selectedRobot = snapshot.robots.find((robot) => robot.id === selectedRobotId);

  if (blockedRobots.length > 0) {
    return `${blockedRobots.length} robot${blockedRobots.length === 1 ? ' is' : 's are'} yielding or blocked while the fleet protects staff traffic and avoids collisions.`;
  }
  if (activeTasks.length > snapshot.robots.length) {
    return `Queue pressure is rising: ${activeTasks.length} active tasks are competing for ${snapshot.robots.length} robots.`;
  }
  if (selectedRobot?.diagnostics?.targetLabel) {
    return `${selectedRobot.id} is ${selectedRobot.diagnostics.note.toLowerCase()} toward ${selectedRobot.diagnostics.targetLabel}.`;
  }
  return 'The fleet is balancing delivery assignments, staff right-of-way, dynamic doors, and obstacle avoidance in a deterministic hospital scenario.';
}

function clampInteger(value: string, min: number, max: number): number {
  const parsed = Number.parseInt(value, 10);
  if (Number.isNaN(parsed)) return min;
  return Math.max(min, Math.min(max, parsed));
}
