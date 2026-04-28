import {
  circleRectOverlap,
  clamp,
  distance,
  isCircleFree,
  isSegmentFree,
  navigationWaypoint,
  normalizeAngle,
  steerAwayFromObstacles,
} from './geometry';
import type { Rect, RobotState, Scenario, StaffState, TaskState, Vec2 } from './types';

export type RobotObservation = {
  time: number;
  dt: number;
  self: RobotState;
  assignedTask?: TaskState;
  scenario: Scenario;
  staticObstacles: Rect[];
  sensedStaff: StaffState[];
  sensedRobots: RobotState[];
  trafficRules: {
    staffYieldRadius: number;
    nearMissRadius: number;
  };
};

export type RobotAction =
  | {
      kind: 'velocity';
      linear: number;
      angular: number;
      note?: string;
    }
  | {
      kind: 'hold';
      reason: string;
    }
  | {
      kind: 'complete-service';
    };

export interface RobotPolicy {
  id: string;
  label: string;
  decide(observation: RobotObservation): RobotAction;
}

export type RobotPolicyId = 'sfm' | 'cbf' | 'mpc';

export const robotPolicyOptions: Array<{ id: RobotPolicyId; label: string }> = [
  { id: 'sfm', label: 'Social Force Model' },
  { id: 'cbf', label: 'Control Barrier Filter' },
  { id: 'mpc', label: 'Predictive MPC' },
];

const targetForTask = (task: TaskState): Vec2 => {
  if (task.status === 'assigned') {
    return task.origin.position;
  }
  return task.destination.position;
};

type ActiveTaskContext = {
  task: TaskState;
  target: Vec2;
  distToTarget: number;
};

const prepareTaskContext = (observation: RobotObservation): RobotAction | ActiveTaskContext => {
  const task = observation.assignedTask;
  if (!task) return { kind: 'hold', reason: 'idle' };

  const target = targetForTask(task);
  const distToTarget = distance(observation.self.position, target);
  if (task.status === 'inService' && distToTarget < 0.8) return { kind: 'complete-service' };
  if (distToTarget < 0.55) return { kind: 'hold', reason: 'arrived' };

  return { task, target, distToTarget };
};

const agentVelocity = (agent: Pick<StaffState, 'speed' | 'theta'>): Vec2 => ({
  x: Math.cos(agent.theta) * agent.speed,
  y: Math.sin(agent.theta) * agent.speed,
});

const closestPointOnRect = (point: Vec2, rect: Rect): Vec2 => ({
  x: clamp(point.x, rect.x - rect.halfW, rect.x + rect.halfW),
  y: clamp(point.y, rect.y - rect.halfH, rect.y + rect.halfH),
});

const nominalTargetCommand = (
  observation: RobotObservation,
  target: Vec2,
  distToTarget: number,
  avoidGain = 0.7,
): RobotAction => {
  const routeTarget = navigationWaypoint(
    observation.self.position,
    target,
    observation.self.radius,
    observation.staticObstacles,
    observation.scenario,
    0.06,
  );
  const desired = {
    x: routeTarget.x - observation.self.position.x,
    y: routeTarget.y - observation.self.position.y,
  };
  const desiredMag = Math.max(0.001, Math.hypot(desired.x, desired.y));
  const avoid = steerAwayFromObstacles(observation.self.position, observation.staticObstacles, avoidGain);
  const headingVector = {
    x: desired.x / desiredMag + avoid.x * 0.4,
    y: desired.y / desiredMag + avoid.y * 0.4,
  };
  const heading = Math.atan2(headingVector.y, headingVector.x);
  const headingError = normalizeAngle(heading - observation.self.theta);
  const alignment = Math.max(0.15, Math.cos(headingError));
  const waypointDistance = distance(observation.self.position, routeTarget);
  const speed = Math.min(observation.self.maxSpeed, distToTarget / 2, Math.max(0.35, waypointDistance) / 1.2) * alignment;

  return {
    kind: 'velocity',
    linear: speed,
    angular: clamp(headingError * 2.6, -2.6, 2.6),
    note: 'Tracking task target with local obstacle steering',
  };
};

export class SocialForcePolicy implements RobotPolicy {
  readonly id: string = 'sfm';
  readonly label: string = 'Social Force Model';

  decide(observation: RobotObservation): RobotAction {
    const context = prepareTaskContext(observation);
    if ('kind' in context) return context;

    const { target, task } = context;
    const { self } = observation;
    const routeTarget = navigationWaypoint(self.position, target, self.radius, observation.staticObstacles, observation.scenario, 0.12);
    const heading = { x: Math.cos(self.theta), y: Math.sin(self.theta) };
    const toTarget = { x: routeTarget.x - self.position.x, y: routeTarget.y - self.position.y };
    const targetDist = Math.max(0.001, Math.hypot(toTarget.x, toTarget.y));
    const desiredDir = { x: toTarget.x / targetDist, y: toTarget.y / targetDist };
    const desiredSpeed = self.maxSpeed * 0.88;
    const tau = 0.55;

    let forceX = (desiredSpeed * desiredDir.x - desiredSpeed * 0.25 * heading.x) / tau;
    let forceY = (desiredSpeed * desiredDir.y - desiredSpeed * 0.25 * heading.y) / tau;

    const addAgentRepulsion = (position: Vec2, radius: number, strength: number, range: number) => {
      const ox = position.x - self.position.x;
      const oy = position.y - self.position.y;
      const d = Math.max(0.001, Math.hypot(ox, oy));
      const combinedRadius = self.radius + radius;
      const away = { x: -ox / d, y: -oy / d };
      const directionToObstacle = { x: ox / d, y: oy / d };
      const forwardWeight = 0.5 + 0.5 * ((1 + heading.x * directionToObstacle.x + heading.y * directionToObstacle.y) / 2);
      const magnitude = strength * Math.exp((combinedRadius - d) / range) * forwardWeight;
      forceX += away.x * magnitude;
      forceY += away.y * magnitude;
    };

    for (const staff of observation.sensedStaff) addAgentRepulsion(staff.position, staff.radius, 6.0, 0.65);
    for (const robot of observation.sensedRobots) addAgentRepulsion(robot.position, robot.radius, 2.4, 0.4);

    const isPickup = task.status === 'assigned';
    const wallRepulsion = steerAwayFromObstacles(self.position, observation.staticObstacles, isPickup ? 1.0 : 1.4);
    const wallMult = isPickup ? 0.5 : 0.9;
    forceX += wallRepulsion.x * wallMult;
    forceY += wallRepulsion.y * wallMult;

    const linear = clamp(forceX * heading.x + forceY * heading.y, 0, self.maxSpeed);
    const angular = clamp((forceX * -heading.y + forceY * heading.x) * 2.0, -2.8, 2.8);

    return {
      kind: 'velocity',
      linear,
      angular,
      note: task.status === 'inService' ? 'SFM delivery with staff and wall repulsion' : 'SFM pickup with staff and wall repulsion',
    };
  }
}

export class ControlBarrierPolicy implements RobotPolicy {
  readonly id: string = 'cbf';
  readonly label: string = 'Control Barrier Filter';

  decide(observation: RobotObservation): RobotAction {
    const context = prepareTaskContext(observation);
    if ('kind' in context) return context;

    const isPickup = context.task.status === 'assigned';
    const nominal = nominalTargetCommand(observation, context.target, context.distToTarget, isPickup ? 0.4 : 0.7);
    if (nominal.kind !== 'velocity') return nominal;

    const { self } = observation;
    const heading = { x: Math.cos(self.theta), y: Math.sin(self.theta) };
    let minLinear = 0;
    let maxLinear = self.maxSpeed;

    const addBarrier = (obstacle: Vec2, radius: number, velocity: Vec2, buffer: number, gamma: number) => {
      const dx = self.position.x - obstacle.x;
      const dy = self.position.y - obstacle.y;
      const safeRadius = self.radius + radius + buffer;
      const h = dx * dx + dy * dy - safeRadius * safeRadius;
      const a = 2 * (dx * heading.x + dy * heading.y);
      const b = -gamma * h + 2 * dx * velocity.x + 2 * dy * velocity.y;
      if (Math.abs(a) < 1e-6) {
        if (b > 0) maxLinear = Math.min(maxLinear, 0);
        return;
      }
      const bound = b / a;
      if (a > 0) minLinear = Math.max(minLinear, bound);
      else maxLinear = Math.min(maxLinear, bound);
    };

    for (const staff of observation.sensedStaff) addBarrier(staff.position, staff.radius, agentVelocity(staff), 0.65, 1.7);
    for (const robot of observation.sensedRobots) addBarrier(robot.position, robot.radius, { x: 0, y: 0 }, 0.65, 4.5);
    const wallBuffer = isPickup ? 0.02 : 0.1;
    const wallGamma = isPickup ? 1.5 : 2.0;
    const wallTrigger = isPickup ? 1.4 : 1.8;
    for (const rect of observation.staticObstacles) {
      const point = closestPointOnRect(self.position, rect);
      if (distance(point, self.position) < wallTrigger) addBarrier(point, 0.12, { x: 0, y: 0 }, wallBuffer, wallGamma);
    }

    const linear = minLinear > maxLinear ? 0 : clamp(nominal.linear, minLinear, maxLinear);
    return {
      ...nominal,
      linear,
      note: linear < nominal.linear * 0.75 ? 'CBF slowed to preserve clearance' : 'CBF tracking nominal route',
    };
  }
}

export class PredictiveMpcPolicy implements RobotPolicy {
  readonly id: string = 'mpc';
  readonly label: string = 'Predictive MPC';

  decide(observation: RobotObservation): RobotAction {
    const context = prepareTaskContext(observation);
    if ('kind' in context) return context;

    const isPickup = context.task.status === 'assigned';
    const { self } = observation;
    const routeTarget = navigationWaypoint(self.position, context.target, self.radius, observation.staticObstacles, observation.scenario, 0.12);
    const horizon = 8;
    const dt = Math.max(0.05, observation.dt);
    const linearSamples = [0, 0.25, 0.5, 0.75, 1.0].map((scale) => scale * self.maxSpeed);
    const angularSamples = [-2.2, -1.1, -0.35, 0, 0.35, 1.1, 2.2];
    let best: { linear: number; angular: number; cost: number } | undefined;
    const staticClearance = isPickup ? 0.03 : 0.06;

    for (const linear of linearSamples) {
      for (const angular of angularSamples) {
        let pose = { x: self.position.x, y: self.position.y, theta: self.theta };
        let cost = linear === 0 ? 1.5 : 0; // small standing-still discouragement
        for (let step = 1; step <= horizon; step += 1) {
          const previous = { x: pose.x, y: pose.y };
          pose.theta = normalizeAngle(pose.theta + angular * dt);
          pose.x += Math.cos(pose.theta) * linear * dt;
          pose.y += Math.sin(pose.theta) * linear * dt;

          const p = { x: pose.x, y: pose.y };
          const targetDist = distance(p, routeTarget);
          cost += targetDist * targetDist * 2.0 + (linear * linear + angular * angular * 0.12) * 0.18;

          if (!isCircleFree(p, self.radius + staticClearance, observation.staticObstacles, observation.scenario)) {
            cost = Number.POSITIVE_INFINITY;
            break;
          }
          if (!isSegmentFree(previous, p, self.radius + staticClearance, observation.staticObstacles, observation.scenario)) {
            cost = Number.POSITIVE_INFINITY;
            break;
          }
          if (!Number.isFinite(cost)) break;
          for (const staff of observation.sensedStaff) {
            const staffVelocity = agentVelocity(staff);
            const predicted = {
              x: staff.position.x + staffVelocity.x * dt * step,
              y: staff.position.y + staffVelocity.y * dt * step,
            };
            const clearance = distance(p, predicted) - self.radius - staff.radius;
            if (clearance < 0.85) cost += (0.85 - clearance) * (0.85 - clearance) * 600;
          }
          // Only penalise rollout steps that move closer to a robot than we are now
          for (const robot of observation.sensedRobots) {
            const currentClearance = distance(self.position, robot.position) - self.radius - robot.radius;
            const futureClearance = distance(p, robot.position) - self.radius - robot.radius;
            if (futureClearance < 0.55 && futureClearance < currentClearance) {
              cost += (0.55 - futureClearance) * (0.55 - futureClearance) * 400;
            }
          }
        }
        if (!best || cost < best.cost) best = { linear, angular, cost };
      }
    }

    // Only fall back to CBF when every rollout is physically blocked by obstacles
    if (!best || !Number.isFinite(best.cost)) {
      return new ControlBarrierPolicy().decide(observation);
    }

    return {
      kind: 'velocity',
      linear: best.linear,
      angular: best.angular,
      note: 'MPC sampled rollout selected lowest-cost safe command',
    };
  }
}

export class VisualDemoPolicy extends ControlBarrierPolicy {
  readonly id: string = 'visual-demo-policy';
  readonly label: string = 'Presentation Control Policy';
}

export const createRobotPolicy = (id: RobotPolicyId): RobotPolicy => {
  switch (id) {
    case 'sfm':
      return new SocialForcePolicy();
    case 'mpc':
      return new PredictiveMpcPolicy();
    case 'cbf':
    default:
      return new ControlBarrierPolicy();
  }
};
