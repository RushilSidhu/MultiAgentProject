import type { CollisionBreakdown, SimulationScoreBreakdown } from './types';

export const SCORING_CONSTANTS = {
  alpha: 1.4,
  wallCollisionWeight: 0.042,
  robotCollisionWeight: 0.17,
  humanCollisionMultiplier: 1.0,
} as const;

export type SimulationScoreInput = {
  simDuration: number;
  completionTimes: number[];
  uncompletedTaskCreatedAt: number[];
  collisionBreakdown: CollisionBreakdown;
};

export function computeSimulationScore(input: SimulationScoreInput): SimulationScoreBreakdown {
  const {
    alpha,
    wallCollisionWeight,
    robotCollisionWeight,
    humanCollisionMultiplier,
  } = SCORING_CONSTANTS;
  const duration = Math.max(input.simDuration, Number.EPSILON);
  const completed = input.completionTimes.map((elapsed) => elapsed / duration);
  const uncompleted = input.uncompletedTaskCreatedAt.map(
    (createdAt) => (alpha * Math.max(0, input.simDuration - createdAt)) / duration,
  );
  const effectiveTaskTimes = [...completed, ...uncompleted];
  const taskCost =
    effectiveTaskTimes.length === 0
      ? 1
      : effectiveTaskTimes.reduce((sum, value) => sum + value, 0) / effectiveTaskTimes.length;
  const wallPenalty = wallCollisionWeight * input.collisionBreakdown.robotWall;
  const robotPenalty = robotCollisionWeight * input.collisionBreakdown.robotRobot;
  const humanMultiplier = 1 + humanCollisionMultiplier * input.collisionBreakdown.robotStaff;
  const score = (taskCost + wallPenalty + robotPenalty) * humanMultiplier;

  return {
    score,
    taskCost,
    wallPenalty,
    robotPenalty,
    humanMultiplier,
  };
}
