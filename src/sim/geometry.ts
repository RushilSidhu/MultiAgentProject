import type { Door, DoorState, Rect, Vec2 } from './types';

export const distance = (a: Vec2, b: Vec2): number => Math.hypot(a.x - b.x, a.y - b.y);

export const clamp = (value: number, min: number, max: number): number =>
  Math.max(min, Math.min(max, value));

export const normalizeAngle = (angle: number): number => {
  let value = angle;
  while (value > Math.PI) value -= Math.PI * 2;
  while (value < -Math.PI) value += Math.PI * 2;
  return value;
};

export const circleRectOverlap = (center: Vec2, radius: number, rect: Rect): boolean => {
  const closestX = clamp(center.x, rect.x - rect.halfW, rect.x + rect.halfW);
  const closestY = clamp(center.y, rect.y - rect.halfH, rect.y + rect.halfH);
  return Math.hypot(center.x - closestX, center.y - closestY) <= radius;
};

export const pointInRect = (point: Vec2, rect: Pick<Rect, 'x' | 'y' | 'halfW' | 'halfH'>): boolean =>
  point.x >= rect.x - rect.halfW &&
  point.x <= rect.x + rect.halfW &&
  point.y >= rect.y - rect.halfH &&
  point.y <= rect.y + rect.halfH;

export const isCircleFree = (
  center: Vec2,
  radius: number,
  obstacles: Rect[],
  world: { width: number; height: number },
): boolean => {
  if (
    center.x < radius ||
    center.y < radius ||
    center.x > world.width - radius ||
    center.y > world.height - radius
  ) {
    return false;
  }
  return !obstacles.some((rect) => circleRectOverlap(center, radius, rect));
};

export const computeDoorStates = (
  doors: Door[],
  agents: Array<{ position: Vec2 }>,
): DoorState[] =>
  doors.map((door) => ({
    id: door.id,
    // Doorways are always-open passages; no proximity trigger logic.
    isOpen: true,
  }));

export const effectiveObstacles = (
  staticObstacles: Rect[],
  doors: Door[],
  doorStates: DoorState[],
): Rect[] => {
  return staticObstacles;
};

export const steerAwayFromObstacles = (
  position: Vec2,
  obstacles: Rect[],
  senseRadius: number,
): Vec2 => {
  const steer = { x: 0, y: 0 };
  for (const rect of obstacles) {
    const closestX = clamp(position.x, rect.x - rect.halfW, rect.x + rect.halfW);
    const closestY = clamp(position.y, rect.y - rect.halfH, rect.y + rect.halfH);
    const dx = position.x - closestX;
    const dy = position.y - closestY;
    const dist = Math.max(0.001, Math.hypot(dx, dy));
    if (dist < senseRadius) {
      const gain = (senseRadius - dist) / senseRadius;
      steer.x += (dx / dist) * gain;
      steer.y += (dy / dist) * gain;
    }
  }
  const mag = Math.hypot(steer.x, steer.y);
  return mag > 1 ? { x: steer.x / mag, y: steer.y / mag } : steer;
};
