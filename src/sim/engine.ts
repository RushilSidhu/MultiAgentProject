import {
  circleRectOverlap,
  clamp,
  computeDoorStates,
  distance,
  effectiveObstacles,
  isCircleFree,
  isSegmentFree,
  navigationWaypoint,
  normalizeAngle,
  steerAwayFromObstacles,
} from './geometry';
import type { RobotAction, RobotObservation, RobotPolicy } from './policies';
import { SeededRandom } from './random';
import { computeSimulationScore } from './scoring';
import type {
  CollisionBreakdown,
  DoorState,
  Metrics,
  Poi,
  Rect,
  RobotDiagnostics,
  RobotState,
  Room,
  Scenario,
  SimulationEvent,
  SimulationSnapshot,
  StaffRole,
  StaffState,
  TaskState,
  Vec2,
} from './types';

export type SimulationConfig = {
  seed: number;
  robotCount: number;
  staffCount: number;
  cartStaffCount: number;
  taskMeanInterval: number;
  dt: number;
};

type NearMissEpisode = {
  key: string;
  active: boolean;
};

type AgentCircle = {
  id: string;
  position: Vec2;
  radius: number;
};

const STATIC_CLEARANCE_BUFFER = 0.12;
const AGENT_HARD_CLEARANCE_BUFFER = 0.04;
const WALL_PUSHBACK_GAP = 0.06;

export class SimulationEngine {
  private readonly rng: SeededRandom;
  private readonly events: SimulationEvent[] = [];
  private readonly completedDurations: number[] = [];
  private readonly nearMissEpisodes = new Map<string, NearMissEpisode>();
  private robots: RobotState[] = [];
  private staff: StaffState[] = [];
  private tasks: TaskState[] = [];
  private doorStates: DoorState[] = [];
  private nextTaskAt = 0;
  private taskCounter = 1;
  private time = 0;
  private busyRobotSeconds = 0;
  private collisionCount = 0;
  private collisionBreakdown: CollisionBreakdown = {
    robotStaff: 0,
    robotRobot: 0,
    robotWall: 0,
  };
  private nearMissCount = 0;

  constructor(
    private readonly scenario: Scenario,
    private readonly policy: RobotPolicy,
    private readonly config: SimulationConfig,
  ) {
    this.rng = new SeededRandom(config.seed);
    this.robots = this.createRobots();
    this.staff = this.createStaff();
    this.updateDoorStates();
    this.nextTaskAt = this.sampleTaskInterval();
  }

  step(dt = this.config.dt): SimulationSnapshot {
    this.time += dt;
    if (this.time >= this.nextTaskAt) {
      this.createTask();
      this.nextTaskAt = this.time + this.sampleTaskInterval();
    }

    this.assignQueuedTasks();
    this.updateDoorStates();
    this.updateStaff(dt);
    this.updateDoorStates();
    this.updateRobots(dt);
    this.updateDoorStates();
    this.updateTaskService();
    this.assertAssignmentIntegrity();
    this.detectSafetyEvents();
    this.busyRobotSeconds += this.robots.filter((robot) => robot.status !== 'idle').length * dt;
    return this.snapshot();
  }

  runFor(seconds: number): SimulationSnapshot {
    const steps = Math.ceil(seconds / this.config.dt);
    let snap = this.snapshot();
    for (let i = 0; i < steps; i += 1) {
      snap = this.step();
    }
    return snap;
  }

  snapshot(): SimulationSnapshot {
    return {
      time: this.time,
      seed: this.config.seed,
      scenario: this.scenario,
      doorStates: this.doorStates.map((door) => ({ ...door })),
      robots: this.robots.map((robot) => ({
        ...robot,
        position: { ...robot.position },
        trail: [...robot.trail],
        diagnostics: robot.diagnostics
          ? {
              ...robot.diagnostics,
              target: robot.diagnostics.target ? { ...robot.diagnostics.target } : undefined,
              sensedStaffIds: [...robot.diagnostics.sensedStaffIds],
              sensedRobotIds: [...robot.diagnostics.sensedRobotIds],
            }
          : undefined,
      })),
      staff: this.staff.map((person) => ({ ...person, position: { ...person.position }, goal: { ...person.goal }, trail: [...person.trail] })),
      tasks: this.tasks.map((task) => ({ ...task })),
      events: this.events.slice(-120),
      metrics: this.metrics(),
    };
  }

  private createRobots(): RobotState[] {
    const robots: RobotState[] = [];
    for (let index = 0; index < this.config.robotCount; index += 1) {
      const robot: RobotState = {
        id: `R${index + 1}`,
        position: this.sampleSpawn('robot', 0.7, robots),
        theta: this.rng.range(-Math.PI, Math.PI),
        radius: 0.42,
        maxSpeed: 1.15,
        status: 'idle',
        trail: [],
      };
      robots.push(robot);
    }
    return robots;
  }

  private currentObstacles(): Rect[] {
    return effectiveObstacles(this.scenario.obstacles, this.scenario.doors, this.doorStates);
  }

  private updateDoorStates(): void {
    this.doorStates = computeDoorStates(this.scenario.doors, [...this.robots, ...this.staff]);
  }

  private createStaff(): StaffState[] {
    const total = this.config.staffCount + this.config.cartStaffCount;
    const staff: StaffState[] = [];
    for (let index = 0; index < total; index += 1) {
      const hasCart = index >= this.config.staffCount;
      const radius = hasCart ? 0.62 : 0.32;
      const position = this.sampleSpawn('staff', radius, [...this.robots, ...staff]);
      const role: StaffRole = hasCart ? 'cart_nurse' : index === 0 ? 'charge_nurse' : 'floor_nurse';
      const startRoom = this.roomForPosition(position);
      const destination = this.pickNextStaffDestination(role, startRoom?.id);
      staff.push({
        id: hasCart ? `C${index - this.config.staffCount + 1}` : `N${index + 1}`,
        position,
        theta: this.rng.range(-Math.PI, Math.PI),
        radius,
        speed: hasCart ? 0.58 : 0.78,
        status: 'walking',
        role,
        state: 'transit',
        hasCart,
        goal: this.jitterPoi(destination),
        goalRoomId: destination.roomId,
        workRoomId: startRoom?.id ?? destination.roomId,
        workRemaining: this.sampleWorkDuration(role),
        microMoveRemaining: this.rng.range(1.2, 4.0),
        intent: `heading to ${destination.label}`,
        dwellRemaining: 0,
        trail: [],
      });
    }
    return staff;
  }

  private sampleSpawn(kind: 'robot' | 'staff', radius: number, existingAgents: AgentCircle[] = []): Vec2 {
    const zones = this.scenario.spawnZones.filter((zone) => zone.agentKinds.includes(kind));
    for (let attempt = 0; attempt < 400; attempt += 1) {
      const zone = this.rng.weighted(zones, (candidate) => candidate.weight);
      const candidate = {
        x: this.rng.range(zone.x - zone.halfW, zone.x + zone.halfW),
        y: this.rng.range(zone.y - zone.halfH, zone.y + zone.halfH),
      };
      const closedDoorObstacles = effectiveObstacles(this.scenario.obstacles, this.scenario.doors, []);
      if (
        isCircleFree(candidate, radius, closedDoorObstacles, this.scenario) &&
        this.isClearOfAgents(candidate, radius, undefined, AGENT_HARD_CLEARANCE_BUFFER, undefined, existingAgents)
      ) {
        return candidate;
      }
    }
    return { x: this.scenario.width / 2, y: this.scenario.height / 2 };
  }

  private allAgentCircles(): AgentCircle[] {
    return [
      ...this.robots.map((agent) => ({ id: agent.id, position: agent.position, radius: agent.radius })),
      ...this.staff.map((agent) => ({ id: agent.id, position: agent.position, radius: agent.radius })),
    ];
  }

  private isClearOfAgents(
    center: Vec2,
    radius: number,
    selfId?: string,
    buffer = AGENT_HARD_CLEARANCE_BUFFER,
    currentPosition?: Vec2,
    agents = this.allAgentCircles(),
  ): boolean {
    for (const agent of agents) {
      if (agent.id === selfId) continue;
      const candidateClearance = distance(center, agent.position) - radius - agent.radius;
      if (candidateClearance >= buffer) continue;

      const currentClearance = currentPosition
        ? distance(currentPosition, agent.position) - radius - agent.radius
        : Number.POSITIVE_INFINITY;
      if (candidateClearance <= currentClearance + 1e-6) return false;
    }
    return true;
  }

  private isOccupancyFree(
    center: Vec2,
    radius: number,
    selfId: string,
    obstacles: Rect[],
    currentPosition?: Vec2,
  ): boolean {
    return (
      isCircleFree(center, radius + STATIC_CLEARANCE_BUFFER, obstacles, this.scenario) &&
      this.isClearOfAgents(center, radius, selfId, AGENT_HARD_CLEARANCE_BUFFER, currentPosition)
    );
  }

  private isSweptOccupancyFree(start: Vec2, end: Vec2, radius: number, selfId: string, obstacles: Rect[]): boolean {
    return (
      isSegmentFree(start, end, radius + STATIC_CLEARANCE_BUFFER, obstacles, this.scenario) &&
      this.isClearOfAgents(end, radius, selfId, AGENT_HARD_CLEARANCE_BUFFER, start)
    );
  }

  private staticClearance(center: Vec2, radius: number, obstacles: Rect[]): number {
    let clearance = Math.min(center.x, center.y, this.scenario.width - center.x, this.scenario.height - center.y) - radius;
    for (const rect of obstacles) {
      const closestX = clamp(center.x, rect.x - rect.halfW, rect.x + rect.halfW);
      const closestY = clamp(center.y, rect.y - rect.halfH, rect.y + rect.halfH);
      clearance = Math.min(clearance, Math.hypot(center.x - closestX, center.y - closestY) - radius);
    }
    return clearance;
  }

  private sampleTaskInterval(): number {
    return Math.max(1, this.rng.range(this.config.taskMeanInterval * 0.45, this.config.taskMeanInterval * 1.55));
  }

  private activeRoomIds(): Set<string> {
    const busy = new Set<string>();
    for (const task of this.tasks) {
      if (task.status !== 'assigned' && task.status !== 'inService') continue;
      busy.add(task.origin.roomId);
      busy.add(task.destination.roomId);
    }
    return busy;
  }

  private createTask(): void {
    const taskClass = this.rng.weighted(this.scenario.taskClasses, (item) => item.priority);
    const busy = this.activeRoomIds();

    const availableOriginIds = taskClass.originPoiIds.filter((id) => !busy.has(this.poiById(id).roomId));
    const availableDestIds = taskClass.destinationPoiIds.filter((id) => !busy.has(this.poiById(id).roomId));
    if (availableOriginIds.length === 0 || availableDestIds.length === 0) return;

    const origin = this.poiById(this.rng.pick(availableOriginIds));
    const destination = this.poiById(this.rng.pick(availableDestIds));
    if (origin.roomId === destination.roomId) return;

    const task: TaskState = {
      id: `T${this.taskCounter++}`,
      classId: taskClass.id,
      label: taskClass.label,
      priority: taskClass.priority,
      origin,
      destination,
      status: 'queued',
      createdAt: this.time,
      deadlineAt: this.time + taskClass.deadlineSeconds,
    };
    this.tasks.push(task);
    this.events.push({ time: this.time, kind: 'task-created', taskId: task.id, label: task.label });
  }

  private assignQueuedTasks(): void {
    const availableRobots = new Set(
      this.robots.filter(
        (robot) => !robot.taskId && (robot.status === 'idle' || robot.status === 'returning'),
      ),
    );
    if (availableRobots.size === 0) return;

    const availableTasks = new Set(
      this.tasks.filter((task) => task.status === 'queued' && !task.assignedRobotId),
    );
    if (availableTasks.size === 0) return;

    while (availableRobots.size > 0 && availableTasks.size > 0) {
      let bestRobot: RobotState | undefined;
      let bestTask: TaskState | undefined;
      let bestDist = Number.POSITIVE_INFINITY;
      for (const robot of availableRobots) {
        for (const task of availableTasks) {
          const d = distance(robot.position, task.origin.position);
          if (d < bestDist) {
            bestDist = d;
            bestRobot = robot;
            bestTask = task;
          }
        }
      }
      if (!bestRobot || !bestTask) break;

      bestRobot.status = 'assigned';
      bestRobot.taskId = bestTask.id;
      bestTask.status = 'assigned';
      bestTask.assignedRobotId = bestRobot.id;
      this.snapThetaToward(bestRobot, bestTask.origin.position);
      this.events.push({ time: this.time, kind: 'task-assigned', taskId: bestTask.id, robotId: bestRobot.id });
      availableRobots.delete(bestRobot);
      availableTasks.delete(bestTask);
    }
    this.assertAssignmentIntegrity();
  }

  private assertAssignmentIntegrity(): void {
    const activeTasks = this.tasks.filter((task) => task.status === 'assigned' || task.status === 'inService');
    const activeTaskIds = new Set(activeTasks.map((task) => task.id));
    const seenRobotTaskIds = new Set<string>();

    for (const robot of this.robots) {
      if (!robot.taskId) continue;
      if (!activeTaskIds.has(robot.taskId)) {
        throw new Error(`Robot ${robot.id} references inactive task ${robot.taskId}`);
      }
      if (seenRobotTaskIds.has(robot.taskId)) {
        throw new Error(`Multiple robots are assigned to task ${robot.taskId}`);
      }
      seenRobotTaskIds.add(robot.taskId);
    }

    for (const task of activeTasks) {
      if (!task.assignedRobotId) {
        throw new Error(`Active task ${task.id} has no assigned robot`);
      }
      const matchingRobots = this.robots.filter((robot) => robot.id === task.assignedRobotId && robot.taskId === task.id);
      if (matchingRobots.length !== 1) {
        throw new Error(`Active task ${task.id} is not owned by exactly one robot`);
      }
    }
  }

  private updateStaff(dt: number): void {
    for (const person of this.staff) {
      switch (person.state) {
        case 'working-in-room':
          this.updateStaffWorking(person, dt);
          break;
        case 'fetch-cart':
        case 'transit':
          if (this.moveStaffTowardGoal(person, dt, 1)) {
            if (person.state === 'fetch-cart') {
              person.intent = 'collected supplies';
              this.beginStaffTransit(person, false);
            } else {
              this.arriveStaffAtDestination(person);
            }
          }
          break;
        case 'dwelling':
          person.status = 'dwelling';
          person.dwellRemaining -= dt;
          if (person.dwellRemaining <= 0) {
            this.beginStaffTransit(person, true);
          }
          break;
      }
    }
  }

  private updateStaffWorking(person: StaffState, dt: number): void {
    person.status = 'walking';
    person.workRemaining -= dt;
    person.microMoveRemaining -= dt;

    const room = this.scenario.rooms.find((item) => item.id === person.workRoomId) ?? this.roomForPosition(person.position);
    if (!room || room.kind === 'corridor' || room.kind === 'connector') {
      this.beginStaffTransit(person, true);
      return;
    }

    if (person.workRemaining <= 0) {
      this.beginStaffTransit(person, true);
      return;
    }

    if (person.microMoveRemaining <= 0 || distance(person.position, person.goal) < 0.25) {
      person.goal = this.samplePointInRoom(room, person.radius);
      person.goalRoomId = room.id;
      person.microMoveRemaining = this.rng.range(2.5, 5.5);
      person.intent = `working in ${room.name}`;
    }

    if (distance(person.position, person.goal) > 0.18) {
      this.moveStaffTowardGoal(person, dt, 0.45);
    } else {
      person.speed = Math.max(0.18, person.speed * 0.9);
    }
  }

  private beginStaffTransit(person: StaffState, allowSupplyStop: boolean): void {
    const currentRoom = this.roomForPosition(person.position)?.id ?? person.workRoomId;
    if (allowSupplyStop && person.role === 'cart_nurse' && this.rng.next() < 0.3) {
      const supply = this.scenario.pois.find((poi) => poi.staffStop && poi.roomId === 'supply');
      if (supply) {
        person.state = 'fetch-cart';
        person.status = 'walking';
        person.goal = this.jitterPoi(supply);
        person.goalRoomId = supply.roomId;
        person.intent = 'fetching cart supplies';
        return;
      }
    }

    const next = this.pickNextStaffDestination(person.role, currentRoom);
    person.state = 'transit';
    person.status = 'walking';
    person.goal = this.jitterPoi(next);
    person.goalRoomId = next.roomId;
    person.intent = `heading to ${next.label}`;
  }

  private arriveStaffAtDestination(person: StaffState): void {
    const room = this.scenario.rooms.find((item) => item.id === person.goalRoomId) ?? this.roomForPosition(person.position);
    if (!room || room.kind === 'corridor' || room.kind === 'connector') {
      person.state = 'dwelling';
      person.status = 'dwelling';
      person.dwellRemaining = this.rng.range(0.8, 2.8);
      person.intent = 'dwelling in corridor';
      return;
    }

    person.state = 'working-in-room';
    person.status = 'walking';
    person.workRoomId = room.id;
    person.workRemaining = this.sampleWorkDuration(person.role);
    person.microMoveRemaining = this.rng.range(1.0, 3.5);
    person.goal = this.samplePointInRoom(room, person.radius);
    person.goalRoomId = room.id;
    person.intent = `working in ${room.name}`;
  }

  private moveStaffTowardGoal(person: StaffState, dt: number, speedScale: number): boolean {
    const distToFinalGoal = distance(person.position, person.goal);
    if (distToFinalGoal < 0.35) {
      person.speed = 0;
      return true;
    }

    const currentObstacles = this.currentObstacles();
    const routeGoal = navigationWaypoint(person.position, person.goal, person.radius, currentObstacles, this.scenario, person.hasCart ? 0.06 : 0.08);
    const toGoal = { x: routeGoal.x - person.position.x, y: routeGoal.y - person.position.y };
    const distToGoal = Math.max(0.001, Math.hypot(toGoal.x, toGoal.y));
    const desired = { x: toGoal.x / distToGoal, y: toGoal.y / distToGoal };
    const obstacleAvoid = steerAwayFromObstacles(person.position, currentObstacles, 1.25);
    const neighborAvoid = this.staffNeighborAvoidance(person);
    const steer = {
      x: desired.x + obstacleAvoid.x * 1.15 + neighborAvoid.x * 0.95,
      y: desired.y + obstacleAvoid.y * 1.15 + neighborAvoid.y * 0.95,
    };
    const steerMag = Math.max(0.001, Math.hypot(steer.x, steer.y));
    const theta = Math.atan2(steer.y / steerMag, steer.x / steerMag);
    const previousTheta = person.theta;
    person.theta = theta;
    const roleSpeed = person.role === 'cart_nurse' ? 0.58 : person.role === 'charge_nurse' ? 0.72 : 0.78;
    const turnSlowdown = Math.max(0.45, Math.cos(normalizeAngle(theta - previousTheta)));
    const waypointSlowdown = distToGoal < 0.9 ? Math.max(0.35, distToGoal / 0.9) : 1;
    person.speed = roleSpeed * speedScale * turnSlowdown * waypointSlowdown;

    const proposed = {
      x: person.position.x + Math.cos(theta) * person.speed * dt,
      y: person.position.y + Math.sin(theta) * person.speed * dt,
    };
    if (this.isSweptOccupancyFree(person.position, proposed, person.radius, person.id, currentObstacles)) {
      person.position = proposed;
      this.recordTrail(person);
      return false;
    }

    const sidestep = [
      theta + Math.PI / 2,
      theta - Math.PI / 2,
      theta + Math.PI,
    ];
    for (const angle of sidestep) {
      const candidate = {
        x: person.position.x + Math.cos(angle) * person.speed * dt * 0.7,
        y: person.position.y + Math.sin(angle) * person.speed * dt * 0.7,
      };
      if (this.isSweptOccupancyFree(person.position, candidate, person.radius, person.id, currentObstacles)) {
        person.theta = angle;
        person.position = candidate;
        this.recordTrail(person);
        return false;
      }
    }

    person.dwellRemaining = this.rng.range(0.4, 1.2);
    person.state = 'dwelling';
    person.status = 'dwelling';
    return false;
  }

  private staffNeighborAvoidance(person: StaffState): Vec2 {
    const steer = { x: 0, y: 0 };
    const addRepulsion = (other: { position: Vec2; radius: number }, strength: number) => {
      const dx = person.position.x - other.position.x;
      const dy = person.position.y - other.position.y;
      const d = Math.max(0.001, Math.hypot(dx, dy));
      const clearance = d - person.radius - other.radius;
      if (clearance > 1.15) return;
      const gain = (1.15 - clearance) / 1.15;
      steer.x += (dx / d) * gain * strength;
      steer.y += (dy / d) * gain * strength;
    };
    for (const other of this.staff) {
      if (other.id !== person.id) addRepulsion(other, 0.9);
    }
    for (const robot of this.robots) addRepulsion(robot, 0.65);
    const mag = Math.hypot(steer.x, steer.y);
    return mag > 1 ? { x: steer.x / mag, y: steer.y / mag } : steer;
  }

  private pickNextStaffDestination(role: StaffRole, currentRoomId?: string): Poi {
    const stops = this.scenario.pois.filter((poi) => poi.staffStop && poi.roomId !== currentRoomId);
    const candidates = stops.length > 0 ? stops : this.scenario.pois.filter((poi) => poi.staffStop);
    return this.rng.weighted(candidates, (poi) => {
      const room = this.scenario.rooms.find((item) => item.id === poi.roomId);
      if (!room) return 1;
      if (role === 'charge_nurse') {
        if (poi.roomId === 'nurse') return 5;
        if (room.kind === 'clinical') return 3;
        if (room.kind === 'corridor') return 2;
        return 1.4;
      }
      if (role === 'cart_nurse') {
        if (poi.roomId === 'supply') return 5;
        if (room.kind === 'clinical' || room.kind === 'treatment') return 2.8;
        return 1.4;
      }
      if (room.kind === 'patient') return 4;
      if (room.kind === 'clinical' || room.kind === 'treatment') return 2.5;
      return 1.2;
    });
  }

  private roomForPosition(position: Vec2): Room | undefined {
    return this.scenario.rooms.find(
      (room) =>
        position.x >= room.x - room.w / 2 &&
        position.x <= room.x + room.w / 2 &&
        position.y >= room.y - room.h / 2 &&
        position.y <= room.y + room.h / 2,
    );
  }

  private samplePointInRoom(room: Room, radius: number): Vec2 {
    const margin = radius + 0.35;
    for (let attempt = 0; attempt < 80; attempt += 1) {
      const point = {
        x: this.rng.range(room.x - room.w / 2 + margin, room.x + room.w / 2 - margin),
        y: this.rng.range(room.y - room.h / 2 + margin, room.y + room.h / 2 - margin),
      };
      if (isCircleFree(point, radius, this.currentObstacles(), this.scenario)) return point;
    }
    return {
      x: clamp(room.x, radius, this.scenario.width - radius),
      y: clamp(room.y, radius, this.scenario.height - radius),
    };
  }

  private sampleWorkDuration(role: StaffRole): number {
    if (role === 'charge_nurse') return this.rng.range(8, 22);
    if (role === 'cart_nurse') return this.rng.range(10, 28);
    return this.rng.range(12, 36);
  }

  private jitterPoi(poi: Poi): Vec2 {
    return {
      x: poi.position.x + this.rng.range(-0.25, 0.25),
      y: poi.position.y + this.rng.range(-0.25, 0.25),
    };
  }

  private tryMoveRobot(robot: RobotState, stepDistance: number, obstacles: Rect[]): boolean {
    if (stepDistance <= 1e-5) return true;

    const forward = robot.theta;
    const currentClearance = this.staticClearance(robot.position, robot.radius, obstacles);
    const candidates = [
      { angle: forward, scale: 1 },
      { angle: forward, scale: 0.7 },
      { angle: forward, scale: 0.45 },
      { angle: forward + Math.PI / 6, scale: 0.75 },
      { angle: forward - Math.PI / 6, scale: 0.75 },
      { angle: forward + Math.PI / 3, scale: 0.55 },
      { angle: forward - Math.PI / 3, scale: 0.55 },
      { angle: forward + Math.PI / 2, scale: 0.45 },
      { angle: forward - Math.PI / 2, scale: 0.45 },
      { angle: forward + Math.PI, scale: 0.35 },
    ];

    for (const candidate of candidates) {
      const distanceForCandidate = stepDistance * candidate.scale;
      const proposed = {
        x: robot.position.x + Math.cos(candidate.angle) * distanceForCandidate,
        y: robot.position.y + Math.sin(candidate.angle) * distanceForCandidate,
      };
      if (candidate.angle !== forward && this.staticClearance(proposed, robot.radius, obstacles) + 0.01 < currentClearance) {
        continue;
      }
      if (this.isSweptOccupancyFree(robot.position, proposed, robot.radius, robot.id, obstacles)) {
        robot.theta = normalizeAngle(candidate.angle);
        robot.position = proposed;
        this.pushAwayFromWalls(robot, obstacles);
        this.recordTrail(robot);
        return true;
      }
    }

    return false;
  }

  private pushAwayFromWalls(robot: RobotState, obstacles: Rect[]): void {
    for (const rect of obstacles) {
      const closestX = clamp(robot.position.x, rect.x - rect.halfW, rect.x + rect.halfW);
      const closestY = clamp(robot.position.y, rect.y - rect.halfH, rect.y + rect.halfH);
      const dx = robot.position.x - closestX;
      const dy = robot.position.y - closestY;
      const d = Math.hypot(dx, dy);
      if (d < 1e-4) continue;
      const gap = d - robot.radius;
      if (gap < WALL_PUSHBACK_GAP) {
        const push = WALL_PUSHBACK_GAP - gap;
        robot.position = {
          x: robot.position.x + (dx / d) * push,
          y: robot.position.y + (dy / d) * push,
        };
      }
    }
  }

  private snapThetaToward(robot: RobotState, target: Vec2): void {
    const obstacles = this.currentObstacles();
    const route = navigationWaypoint(robot.position, target, robot.radius, obstacles, this.scenario, 0.06);
    const dx = route.x - robot.position.x;
    const dy = route.y - robot.position.y;
    if (Math.hypot(dx, dy) > 1e-3) {
      robot.theta = Math.atan2(dy, dx);
    }
  }

  private hallwayZoneForRobot(robot: RobotState): (typeof this.scenario.spawnZones)[0] | undefined {
    return this.scenario.spawnZones.find(
      (zone) =>
        zone.agentKinds.includes('robot') &&
        Math.abs(robot.position.x - zone.x) <= zone.halfW &&
        Math.abs(robot.position.y - zone.y) <= zone.halfH,
    );
  }

  private shuffleInHallway(robot: RobotState, dt: number, obstacles: Rect[], zone: (typeof this.scenario.spawnZones)[0]): void {
    robot.status = 'idle';
    let steerX = 0;
    let steerY = 0;

    const addRepulsion = (pos: Vec2, radius: number, repelRadius: number, strength: number) => {
      const dx = robot.position.x - pos.x;
      const dy = robot.position.y - pos.y;
      const d = Math.max(0.001, Math.hypot(dx, dy));
      const gap = d - robot.radius - radius;
      if (gap < repelRadius) {
        const gain = (repelRadius - gap) / repelRadius;
        steerX += (dx / d) * gain * strength;
        steerY += (dy / d) * gain * strength;
      }
    };

    for (const other of this.robots) {
      if (other.id !== robot.id) addRepulsion(other.position, other.radius, 2.5, 1.0);
    }
    for (const person of this.staff) {
      addRepulsion(person.position, person.radius, 2.0, 0.8);
    }

    // Gentle attractor toward zone centre so robots don't drift to the edges
    const toCentreX = zone.x - robot.position.x;
    const toCentreY = zone.y - robot.position.y;
    const centreDist = Math.hypot(toCentreX, toCentreY);
    if (centreDist > 0.5) {
      const centreGain = Math.min(1.0, centreDist / (zone.halfW * 0.6)) * 0.35;
      steerX += (toCentreX / centreDist) * centreGain;
      steerY += (toCentreY / centreDist) * centreGain;
    }

    const mag = Math.hypot(steerX, steerY);
    if (mag < 0.05) {
      robot.diagnostics = {
        updatedAt: this.time,
        action: 'hold',
        phase: 'idle',
        note: 'Holding in hallway zone',
        sensedStaffIds: [],
        sensedRobotIds: [],
        staffYieldRadius: 1.15,
        nearMissRadius: 0.45,
        linear: 0,
        angular: 0,
      };
      return;
    }

    robot.theta = Math.atan2(steerY, steerX);
    const speed = Math.min(robot.maxSpeed * 0.55, mag * 0.7);
    const moved = this.tryMoveRobot(robot, speed * dt, obstacles);
    robot.diagnostics = {
      updatedAt: this.time,
      action: moved ? 'velocity' : 'hold',
      phase: 'idle',
      note: 'Shuffling in hallway zone',
      sensedStaffIds: this.staff.filter((p) => distance(p.position, robot.position) < 3).map((p) => p.id),
      sensedRobotIds: this.robots
        .filter((r) => r.id !== robot.id && distance(r.position, robot.position) < 3)
        .map((r) => r.id),
      staffYieldRadius: 1.15,
      nearMissRadius: 0.45,
      linear: moved ? speed : 0,
      angular: 0,
    };
  }

  private idleTarget(robot: RobotState): { label: string; position: Vec2 } {
    const currentRoom = this.roomForPosition(robot.position);
    if (currentRoom && currentRoom.kind !== 'corridor' && currentRoom.kind !== 'connector') {
      const hallway1Y = 31;
      const hallway2Y = 20;
      const targetY =
        Math.abs(robot.position.y - hallway1Y) < Math.abs(robot.position.y - hallway2Y)
          ? hallway1Y
          : hallway2Y;
      return { label: 'hallway', position: { x: clamp(robot.position.x, 2, 58), y: targetY } };
    }
    const robotZones = this.scenario.spawnZones.filter((zone) => zone.agentKinds.includes('robot'));
    if (robotZones.length === 0) return { label: 'hallway', position: { x: this.scenario.width / 2, y: this.scenario.height / 2 } };
    robotZones.sort(
      (a, b) =>
        distance(robot.position, { x: a.x, y: a.y }) -
        distance(robot.position, { x: b.x, y: b.y }),
    );
    const zone = robotZones[0];
    return { label: zone.label, position: { x: zone.x, y: zone.y } };
  }

  private updateRobotWithoutTask(robot: RobotState, dt: number, obstacles: Rect[]): void {
    const zone = this.hallwayZoneForRobot(robot);
    if (zone) {
      this.shuffleInHallway(robot, dt, obstacles, zone);
      return;
    }

    const staging = this.idleTarget(robot);
    const distToStaging = distance(robot.position, staging.position);

    const routeTarget = navigationWaypoint(robot.position, staging.position, robot.radius, obstacles, this.scenario, 0.06);
    const desired = {
      x: routeTarget.x - robot.position.x,
      y: routeTarget.y - robot.position.y,
    };
    const desiredMag = Math.max(0.001, Math.hypot(desired.x, desired.y));
    const avoid = steerAwayFromObstacles(robot.position, obstacles, 0.9);
    const heading = Math.atan2(desired.y / desiredMag + avoid.y * 0.3, desired.x / desiredMag + avoid.x * 0.3);

    robot.status = 'returning';
    robot.theta = heading;
    const linear = Math.min(robot.maxSpeed * 0.95, distToStaging / 1.5);
    const moved = this.tryMoveRobot(robot, linear * dt, obstacles);
    robot.diagnostics = {
      updatedAt: this.time,
      action: moved ? 'velocity' : 'hold',
      phase: 'idle',
      note: moved ? `Heading to ${staging.label}` : `Blocked en route to ${staging.label}`,
      target: { ...staging.position },
      targetLabel: staging.label,
      sensedStaffIds: [],
      sensedRobotIds: [],
      staffYieldRadius: 1.15,
      nearMissRadius: 0.45,
      linear: moved ? linear : 0,
      angular: 0,
    };
  }

  private updateRobots(dt: number): void {
    for (const robot of this.robots) {
      const currentObstacles = this.currentObstacles();
      const assignedTask = this.tasks.find((task) => task.id === robot.taskId);
      if (robot.taskId && !assignedTask) {
        throw new Error(`Robot ${robot.id} references missing task ${robot.taskId}`);
      }
      if (!assignedTask) {
        this.updateRobotWithoutTask(robot, dt, currentObstacles);
        continue;
      }
      const sensedStaff = this.staff.filter((person) => distance(person.position, robot.position) < 5);
      const sensedRobots = this.robots.filter((other) => other.id !== robot.id && distance(other.position, robot.position) < 5);
      const trafficRules = { staffYieldRadius: 1.15, nearMissRadius: 0.45 };
      const observation: RobotObservation = {
        time: this.time,
        dt,
        self: robot,
        assignedTask,
        scenario: this.scenario,
        staticObstacles: currentObstacles,
        sensedStaff,
        sensedRobots,
        trafficRules,
      };
      const action = this.policy.decide(observation);
      robot.diagnostics = this.createRobotDiagnostics(robot, assignedTask, action, observation);

      if (action.kind === 'complete-service') {
        this.completeTask(robot, assignedTask);
        continue;
      }
      if (action.kind === 'hold') {
        robot.status = assignedTask ? 'blocked' : 'idle';
        if (assignedTask && action.reason !== 'arrived') {
          this.events.push({ time: this.time, kind: 'robot-blocked', robotId: robot.id, reason: action.reason });
        }
        continue;
      }

      robot.status = assignedTask?.status === 'inService' ? 'servicing' : 'assigned';
      robot.theta = normalizeAngle(robot.theta + action.angular * dt);
      if (!this.tryMoveRobot(robot, action.linear * dt, currentObstacles)) {
        robot.status = 'blocked';
        this.events.push({ time: this.time, kind: 'robot-blocked', robotId: robot.id, reason: 'local motion recovery failed' });
      }
    }
  }

  private createRobotDiagnostics(
    robot: RobotState,
    task: TaskState | undefined,
    action: RobotAction,
    observation: RobotObservation,
  ): RobotDiagnostics {
    const phase = !task ? 'idle' : action.kind === 'complete-service' ? 'complete' : task.status === 'inService' ? 'delivery' : 'pickup';
    const target =
      task && phase !== 'complete'
        ? task.status === 'inService'
          ? task.destination.position
          : task.origin.position
        : undefined;
    const targetLabel =
      task && phase !== 'complete'
        ? task.status === 'inService'
          ? task.destination.label
          : task.origin.label
        : undefined;
    const note =
      action.kind === 'velocity'
        ? action.note ?? `Navigating to ${targetLabel ?? 'target'}`
        : action.kind === 'hold'
          ? `Holding: ${action.reason}`
          : 'Completing delivery at destination';

    return {
      updatedAt: this.time,
      action: action.kind,
      phase,
      note,
      reason: action.kind === 'hold' ? action.reason : undefined,
      taskId: task?.id,
      taskLabel: task?.label,
      target: target ? { ...target } : undefined,
      targetLabel,
      sensedStaffIds: observation.sensedStaff.map((person) => person.id),
      sensedRobotIds: observation.sensedRobots.map((other) => other.id),
      staffYieldRadius: observation.trafficRules.staffYieldRadius,
      nearMissRadius: observation.trafficRules.nearMissRadius,
      linear: action.kind === 'velocity' ? action.linear : 0,
      angular: action.kind === 'velocity' ? action.angular : 0,
    };
  }

  private updateTaskService(): void {
    for (const task of this.tasks) {
      if (task.status !== 'assigned' || !task.assignedRobotId) continue;
      const robot = this.robots.find((candidate) => candidate.id === task.assignedRobotId);
      if (!robot) continue;
      if (distance(robot.position, task.origin.position) < 0.65) {
        task.status = 'inService';
        task.serviceStartedAt = this.time;
        robot.status = 'servicing';
        this.snapThetaToward(robot, task.destination.position);
      }
    }
  }

  private completeTask(robot: RobotState, task?: TaskState): void {
    if (!task || task.status !== 'inService') return;
    task.status = 'completed';
    task.completedAt = this.time;
    robot.status = 'idle';
    robot.taskId = undefined;
    const elapsed = this.time - task.createdAt;
    this.completedDurations.push(elapsed);
    this.events.push({ time: this.time, kind: 'task-completed', taskId: task.id, robotId: robot.id, elapsed });
  }

  private detectSafetyEvents(): void {
    const currentObstacles = this.currentObstacles();
    for (const robot of this.robots) {
      for (const rect of currentObstacles) {
        if (circleRectOverlap(robot.position, robot.radius, rect)) {
          this.collisionCount += 1;
          this.collisionBreakdown.robotWall += 1;
          this.pushAwayFromWalls(robot, currentObstacles);
          this.events.push({ time: this.time, kind: 'collision', a: robot.id, b: rect.id, severity: 'collision' });
        }
      }
    }

    const agents = [
      ...this.robots.map((agent) => ({ id: agent.id, position: agent.position, radius: agent.radius })),
      ...this.staff.map((agent) => ({ id: agent.id, position: agent.position, radius: agent.radius })),
    ];
    for (let i = 0; i < agents.length - 1; i += 1) {
      for (let j = i + 1; j < agents.length; j += 1) {
        const a = agents[i];
        const b = agents[j];
        const clearDist = distance(a.position, b.position) - a.radius - b.radius;
        const key = [a.id, b.id].sort().join(':');
        if (clearDist < 0) {
          this.collisionCount += 1;
          this.recordAgentCollision(a.id, b.id);
          this.events.push({ time: this.time, kind: 'collision', a: a.id, b: b.id, severity: 'collision' });
        } else if (clearDist < 0.35) {
          const episode = this.nearMissEpisodes.get(key);
          if (!episode?.active) {
            this.nearMissCount += 1;
            this.nearMissEpisodes.set(key, { key, active: true });
            this.events.push({ time: this.time, kind: 'collision', a: a.id, b: b.id, severity: 'near-miss' });
          }
        } else {
          const episode = this.nearMissEpisodes.get(key);
          if (episode) episode.active = false;
        }
      }
    }
  }

  private recordAgentCollision(a: string, b: string): void {
    const aIsRobot = this.isRobotId(a);
    const bIsRobot = this.isRobotId(b);
    if (aIsRobot && bIsRobot) {
      this.collisionBreakdown.robotRobot += 1;
      return;
    }
    if (aIsRobot || bIsRobot) {
      this.collisionBreakdown.robotStaff += 1;
    }
  }

  private isRobotId(id: string): boolean {
    return id.startsWith('R');
  }

  private metrics(): Metrics {
    const activeTasks = this.tasks.filter((task) => task.status !== 'completed' && task.status !== 'cancelled').length;
    const uncompletedTaskCreatedAt = this.tasks
      .filter((task) => task.status !== 'completed' && task.status !== 'cancelled')
      .map((task) => task.createdAt);
    const collisionBreakdown = { ...this.collisionBreakdown };
    const completionTimes = [...this.completedDurations];
    const score = computeSimulationScore({
      simDuration: this.time,
      completionTimes,
      uncompletedTaskCreatedAt,
      collisionBreakdown,
    });
    const averageCompletionSeconds =
      this.completedDurations.length === 0
        ? 0
        : this.completedDurations.reduce((sum, value) => sum + value, 0) / this.completedDurations.length;
    const denom = Math.max(1, this.time * this.robots.length);
    return {
      completedTasks: this.completedDurations.length,
      queuedTasks: this.tasks.filter((task) => task.status === 'queued').length,
      activeTasks,
      collisions: this.collisionCount,
      collisionBreakdown,
      nearMisses: this.nearMissCount,
      averageCompletionSeconds,
      robotUtilization: this.busyRobotSeconds / denom,
      completionTimes,
      uncompletedTaskCreatedAt,
      score,
    };
  }

  private poiById(id: string): Poi {
    const match = this.scenario.pois.find((poi) => poi.id === id);
    if (!match) throw new Error(`Unknown POI: ${id}`);
    return match;
  }

  private recordTrail(agent: { position: Vec2; trail: Vec2[] }): void {
    agent.trail.push({ ...agent.position });
    if (agent.trail.length > 80) {
      agent.trail.shift();
    }
  }
}
