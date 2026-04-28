import { describe, expect, it } from 'vitest';
import { hospitalScenario } from '../src/scenarios/hospitalScenario';
import { circleRectOverlap, computeDoorStates, effectiveObstacles, isCircleFree } from '../src/sim/geometry';
import { SimulationEngine, type SimulationConfig } from '../src/sim/engine';
import { createRobotPolicy, robotPolicyOptions, VisualDemoPolicy, type RobotPolicy } from '../src/sim/policies';
import { computeSimulationScore } from '../src/sim/scoring';
import type { Poi, Rect, Scenario, SpawnZone } from '../src/sim/types';

const config: SimulationConfig = {
  seed: 17,
  robotCount: 3,
  staffCount: 2,
  cartStaffCount: 1,
  taskMeanInterval: 2,
  dt: 0.1,
};

const summarize = (engine: SimulationEngine) => {
  const snapshot = engine.runFor(20);
  return {
    time: snapshot.time,
    robots: snapshot.robots.map((robot) => ({
      id: robot.id,
      x: Number(robot.position.x.toFixed(3)),
      y: Number(robot.position.y.toFixed(3)),
      status: robot.status,
    })),
    tasks: snapshot.tasks.map((task) => ({
      id: task.id,
      status: task.status,
      assignedRobotId: task.assignedRobotId,
    })),
    metrics: snapshot.metrics,
  };
};

const exactSpawn = (id: string, x: number, y: number, agentKinds: SpawnZone['agentKinds']): SpawnZone => ({
  id,
  label: id,
  x,
  y,
  halfW: 0,
  halfH: 0,
  weight: 1,
  agentKinds,
});

const poi = (id: string, x: number, y: number, taskTarget = true, staffStop = false): Poi => ({
  id,
  label: id,
  roomId: 'test-room',
  position: { x, y },
  taskTarget,
  staffStop,
});

const testScenario = (options: {
  obstacles?: Rect[];
  robotSpawn: { x: number; y: number };
  staffSpawn?: { x: number; y: number };
  pickup?: { x: number; y: number };
  dropoff?: { x: number; y: number };
}): Scenario => {
  const pickup = options.pickup ?? { x: 6.5, y: 3 };
  const dropoff = options.dropoff ?? { x: 6.8, y: 3 };
  const pois = [
    poi('pickup', pickup.x, pickup.y),
    poi('dropoff', dropoff.x, dropoff.y),
    poi('staff-stop', options.staffSpawn?.x ?? 4, options.staffSpawn?.y ?? 3, false, true),
  ];
  return {
    id: 'local-motion-regression',
    name: 'Local Motion Regression',
    width: 8,
    height: 6,
    rooms: [{ id: 'test-room', name: 'Test Room', kind: 'corridor', x: 4, y: 3, w: 8, h: 6 }],
    obstacles: options.obstacles ?? [],
    doors: [],
    pois,
    spawnZones: [
      exactSpawn('robot-spawn', options.robotSpawn.x, options.robotSpawn.y, ['robot']),
      ...(options.staffSpawn ? [exactSpawn('staff-spawn', options.staffSpawn.x, options.staffSpawn.y, ['staff'])] : []),
    ],
    taskClasses: [
      {
        id: 'test-task',
        label: 'Test Task',
        priority: 1,
        serviceSeconds: 1,
        deadlineSeconds: 30,
        originPoiIds: ['pickup'],
        destinationPoiIds: ['dropoff'],
      },
    ],
  };
};

const eastCommandPolicy: RobotPolicy = {
  id: 'east-command',
  label: 'East Command',
  decide(observation) {
    if (!observation.assignedTask) return { kind: 'hold', reason: 'idle' };
    return {
      kind: 'velocity',
      linear: observation.self.maxSpeed,
      angular: -observation.self.theta / observation.dt,
      note: 'Commanding a direct eastward route',
    };
  },
};

const minAgentClearance = (snapshot: ReturnType<SimulationEngine['snapshot']>): number => {
  const agents = [
    ...snapshot.robots.map((agent) => ({ id: agent.id, position: agent.position, radius: agent.radius })),
    ...snapshot.staff.map((agent) => ({ id: agent.id, position: agent.position, radius: agent.radius })),
  ];
  let minClearance = Number.POSITIVE_INFINITY;
  for (let i = 0; i < agents.length - 1; i += 1) {
    for (let j = i + 1; j < agents.length; j += 1) {
      const a = agents[i];
      const b = agents[j];
      minClearance = Math.min(minClearance, Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y) - a.radius - b.radius);
    }
  }
  return minClearance;
};

const expectStrictActiveAssignments = (snapshot: ReturnType<SimulationEngine['snapshot']>) => {
  const activeTasks = snapshot.tasks.filter((task) => task.status === 'assigned' || task.status === 'inService');
  const robotTaskIds = snapshot.robots.flatMap((robot) => (robot.taskId ? [robot.taskId] : []));

  expect(new Set(robotTaskIds).size).toBe(robotTaskIds.length);
  for (const task of activeTasks) {
    expect(task.assignedRobotId).toBeDefined();
    expect(snapshot.robots.filter((robot) => robot.id === task.assignedRobotId && robot.taskId === task.id)).toHaveLength(1);
  }
  for (const taskId of robotTaskIds) {
    expect(activeTasks.some((task) => task.id === taskId)).toBe(true);
  }
};

describe('simulation engine', () => {
  it('is deterministic for a fixed seed and policy', () => {
    const a = new SimulationEngine(hospitalScenario, new VisualDemoPolicy(), config);
    const b = new SimulationEngine(hospitalScenario, new VisualDemoPolicy(), config);

    expect(summarize(a)).toEqual(summarize(b));
  });

  it('is deterministic for each robot control strategy', () => {
    for (const option of robotPolicyOptions) {
      const a = new SimulationEngine(hospitalScenario, createRobotPolicy(option.id), config);
      const b = new SimulationEngine(hospitalScenario, createRobotPolicy(option.id), config);

      expect(summarize(a)).toEqual(summarize(b));
    }
  });

  it('initializes room-aware nurse state', () => {
    const engine = new SimulationEngine(hospitalScenario, createRobotPolicy('cbf'), config);
    const snapshot = engine.snapshot();

    expect(snapshot.staff.every((person) => person.role.endsWith('_nurse'))).toBe(true);
    expect(snapshot.staff.every((person) => person.state === 'transit')).toBe(true);
    expect(snapshot.staff.every((person) => person.workRoomId.length > 0)).toBe(true);
    expect(snapshot.staff.every((person) => person.intent && person.intent.length > 0)).toBe(true);
  });

  it('creates and assigns lifecycle-driven tasks', () => {
    const engine = new SimulationEngine(hospitalScenario, new VisualDemoPolicy(), config);
    const snapshot = engine.runFor(8);

    expect(snapshot.tasks.length).toBeGreaterThan(0);
    expect(snapshot.tasks.some((task) => task.status === 'assigned' || task.status === 'inService')).toBe(true);
    expect(snapshot.events.some((event) => event.kind === 'task-created')).toBe(true);
    expect(snapshot.events.some((event) => event.kind === 'task-assigned')).toBe(true);
  });

  it('reuses a robot for multiple tasks after completion', () => {
    const engine = new SimulationEngine(testScenario({ robotSpawn: { x: 6.1, y: 3 } }), new VisualDemoPolicy(), {
      seed: 9,
      robotCount: 1,
      staffCount: 0,
      cartStaffCount: 0,
      taskMeanInterval: 1,
      dt: 0.1,
    });

    const snapshot = engine.runFor(10);
    const completedEvents = snapshot.events.filter((event) => event.kind === 'task-completed');

    expect(snapshot.metrics.completedTasks).toBeGreaterThan(1);
    expect(new Set(completedEvents.map((event) => event.taskId)).size).toBeGreaterThan(1);
    expect(new Set(completedEvents.map((event) => event.robotId))).toEqual(new Set(['R1']));
  });

  it('keeps active task ownership strict while dispatching', () => {
    const engine = new SimulationEngine(hospitalScenario, new VisualDemoPolicy(), config);

    for (let step = 0; step < 120; step += 1) {
      expectStrictActiveAssignments(engine.step());
    }
  });

  it('returns an available robot toward staging after completing a task', () => {
    const staging = { x: 1.2, y: 3 };
    const engine = new SimulationEngine(
      testScenario({
        robotSpawn: staging,
        pickup: { x: 6.2, y: 3 },
        dropoff: { x: 6.8, y: 3 },
      }),
      new VisualDemoPolicy(),
      {
        seed: 12,
        robotCount: 1,
        staffCount: 0,
        cartStaffCount: 0,
        taskMeanInterval: 20,
        dt: 0.1,
      },
    );

    let afterCompletion = engine.snapshot();
    for (let step = 0; step < 800 && afterCompletion.metrics.completedTasks === 0; step += 1) {
      afterCompletion = engine.step();
    }
    expect(afterCompletion.metrics.completedTasks).toBe(1);

    const distanceAfterCompletion = Math.hypot(afterCompletion.robots[0].position.x - staging.x, afterCompletion.robots[0].position.y - staging.y);
    const afterReturn = engine.runFor(2);
    const distanceAfterReturn = Math.hypot(afterReturn.robots[0].position.x - staging.x, afterReturn.robots[0].position.y - staging.y);

    expect(afterReturn.robots[0].taskId).toBeUndefined();
    expect(afterReturn.robots[0].status).toBe('returning');
    expect(distanceAfterReturn).toBeLessThan(distanceAfterCompletion);
  });

  it('exposes deterministic robot diagnostics for presentation overlays', () => {
    const engine = new SimulationEngine(hospitalScenario, new VisualDemoPolicy(), config);
    const snapshot = engine.runFor(8);
    const activeRobot = snapshot.robots.find((robot) => robot.diagnostics?.taskId);

    expect(activeRobot?.diagnostics).toMatchObject({
      staffYieldRadius: 1.15,
      nearMissRadius: 0.45,
    });
    expect(activeRobot?.diagnostics?.note.length).toBeGreaterThan(0);
    expect(activeRobot?.diagnostics?.sensedStaffIds).toBeInstanceOf(Array);
    expect(activeRobot?.diagnostics?.sensedRobotIds).toBeInstanceOf(Array);
  });

  it('keeps robots from accepting moves that overlap staff', () => {
    const engine = new SimulationEngine(testScenario({ robotSpawn: { x: 2.1, y: 3 }, staffSpawn: { x: 3, y: 3 } }), eastCommandPolicy, {
      seed: 3,
      robotCount: 1,
      staffCount: 1,
      cartStaffCount: 0,
      taskMeanInterval: 1,
      dt: 0.1,
    });

    const snapshot = engine.runFor(3);

    expect(snapshot.metrics.collisionBreakdown.robotStaff).toBe(0);
    expect(minAgentClearance(snapshot)).toBeGreaterThanOrEqual(0);
  });

  it('uses local recovery instead of sticking on a small wall obstacle', () => {
    const wall: Rect = { id: 'test-wall', kind: 'wall', x: 3, y: 3, halfW: 0.12, halfH: 0.55 };
    const engine = new SimulationEngine(testScenario({ robotSpawn: { x: 2.2, y: 3 }, obstacles: [wall] }), eastCommandPolicy, {
      seed: 4,
      robotCount: 1,
      staffCount: 0,
      cartStaffCount: 0,
      taskMeanInterval: 1,
      dt: 0.1,
    });

    const snapshot = engine.runFor(5);
    const robot = snapshot.robots[0];

    expect(snapshot.metrics.collisionBreakdown.robotWall).toBe(0);
    expect(robot.status).not.toBe('down');
    expect(Math.abs(robot.position.y - 3)).toBeGreaterThan(0.05);
  });
});

describe('simulation scoring', () => {
  it('computes the decomposed scalar score from task and collision terms', () => {
    const score = computeSimulationScore({
      simDuration: 100,
      completionTimes: [20, 40],
      uncompletedTaskCreatedAt: [50],
      collisionBreakdown: {
        robotStaff: 1,
        robotRobot: 1,
        robotWall: 2,
      },
    });

    expect(score.taskCost).toBeCloseTo((0.2 + 0.4 + 0.7) / 3);
    expect(score.wallPenalty).toBeCloseTo(0.084);
    expect(score.robotPenalty).toBeCloseTo(0.17);
    expect(score.humanMultiplier).toBe(2);
    expect(score.score).toBeCloseTo(1.3746666667);
  });

  it('uses maximum normalized task cost when no task has spawned', () => {
    const score = computeSimulationScore({
      simDuration: 0,
      completionTimes: [],
      uncompletedTaskCreatedAt: [],
      collisionBreakdown: {
        robotStaff: 0,
        robotRobot: 0,
        robotWall: 0,
      },
    });

    expect(score).toMatchObject({
      score: 1,
      taskCost: 1,
      wallPenalty: 0,
      robotPenalty: 0,
      humanMultiplier: 1,
    });
  });
});

describe('scenario geometry', () => {
  it('has valid task targets and non-empty spawn zones', () => {
    expect(hospitalScenario.pois.filter((poi) => poi.taskTarget).length).toBeGreaterThan(8);
    expect(hospitalScenario.spawnZones.filter((zone) => zone.agentKinds.includes('robot')).length).toBeGreaterThan(0);
    expect(hospitalScenario.spawnZones.filter((zone) => zone.agentKinds.includes('staff')).length).toBeGreaterThan(0);
  });

  it('removes admin rooms and defines sliding room doors', () => {
    expect(hospitalScenario.rooms.some((room) => room.id.includes('admin'))).toBe(false);
    expect(hospitalScenario.obstacles.some((item) => item.id.includes('admin'))).toBe(false);
    expect(hospitalScenario.doors.length).toBeGreaterThanOrEqual(18);
    expect(hospitalScenario.doors.every((door) => door.panel.kind === 'wall')).toBe(true);
  });

  it('treats doorways as always-open passages', () => {
    const door = hospitalScenario.doors.find((item) => item.id === 'door-p101');
    expect(door).toBeDefined();
    const noAgentDoorStates = computeDoorStates(hospitalScenario.doors, []);
    const withAgentDoorStates = computeDoorStates(hospitalScenario.doors, [{ position: { x: door!.panel.x, y: door!.panel.y - 1 } }]);
    const obstaclesNoAgent = effectiveObstacles(hospitalScenario.obstacles, hospitalScenario.doors, noAgentDoorStates);
    const obstaclesWithAgent = effectiveObstacles(hospitalScenario.obstacles, hospitalScenario.doors, withAgentDoorStates);

    expect(noAgentDoorStates.every((item) => item.isOpen)).toBe(true);
    expect(withAgentDoorStates.every((item) => item.isOpen)).toBe(true);
    expect(obstaclesNoAgent.some((item) => item.id === door!.panel.id)).toBe(false);
    expect(obstaclesWithAgent.some((item) => item.id === door!.panel.id)).toBe(false);
  });

  it('detects circle obstacle overlap and free space', () => {
    const wall = hospitalScenario.obstacles.find((item) => item.id === 'wall-west');
    expect(wall).toBeDefined();
    expect(circleRectOverlap({ x: 0.2, y: 20 }, 0.5, wall!)).toBe(true);
    expect(isCircleFree({ x: 30, y: 20 }, 0.4, hospitalScenario.obstacles, hospitalScenario)).toBe(true);
  });

  it('has physical door openings in wall geometry', () => {
    const topDoorCenter = { x: 5, y: 33 };
    const nearbyWall = { x: 10, y: 33 };
    expect(isCircleFree(topDoorCenter, 0.4, hospitalScenario.obstacles, hospitalScenario)).toBe(true);
    expect(isCircleFree(nearbyWall, 0.4, hospitalScenario.obstacles, hospitalScenario)).toBe(false);
  });

  it('keeps door definitions and door states aligned', () => {
    const engine = new SimulationEngine(hospitalScenario, new VisualDemoPolicy(), config);
    const snapshot = engine.snapshot();

    expect(hospitalScenario.doors.length).toBeGreaterThan(0);
    expect(snapshot.doorStates).toHaveLength(hospitalScenario.doors.length);
    expect(snapshot.doorStates.map((door) => door.id).sort()).toEqual(hospitalScenario.doors.map((door) => door.id).sort());
  });
});
