import type { Door, DoorState, Rect, Scenario, Vec2 } from './types';

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

export const isSegmentFree = (
  start: Vec2,
  end: Vec2,
  radius: number,
  obstacles: Rect[],
  world: { width: number; height: number },
): boolean => {
  if (!isCircleFree(start, radius, obstacles, world) || !isCircleFree(end, radius, obstacles, world)) return false;
  return !obstacles.some((rect) => segmentIntersectsExpandedRect(start, end, rect, radius));
};

const segmentIntersectsExpandedRect = (start: Vec2, end: Vec2, rect: Rect, padding: number): boolean => {
  const minX = rect.x - rect.halfW - padding;
  const maxX = rect.x + rect.halfW + padding;
  const minY = rect.y - rect.halfH - padding;
  const maxY = rect.y + rect.halfH + padding;
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  let tMin = 0;
  let tMax = 1;

  const clip = (p: number, q: number): boolean => {
    if (Math.abs(p) < 1e-9) return q >= 0;
    const t = q / p;
    if (p < 0) {
      if (t > tMax) return false;
      if (t > tMin) tMin = t;
    } else {
      if (t < tMin) return false;
      if (t < tMax) tMax = t;
    }
    return true;
  };

  return clip(-dx, start.x - minX) && clip(dx, maxX - start.x) && clip(-dy, start.y - minY) && clip(dy, maxY - start.y);
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

const dedupePoints = (points: Vec2[]): Vec2[] => {
  const seen = new Set<string>();
  const unique: Vec2[] = [];
  for (const point of points) {
    const key = `${point.x.toFixed(3)}:${point.y.toFixed(3)}`;
    if (seen.has(key)) continue;
    seen.add(key);
    unique.push(point);
  }
  return unique;
};

type RouteGraph = {
  nodes: Vec2[];
  edges: number[][];
};

const routeGraphCache = new Map<string, RouteGraph>();
const waypointCache = new Map<string, Vec2>();

const targetApproachPoints = (target: Vec2): Vec2[] => {
  const directions = [
    { x: 1, y: 0 },
    { x: -1, y: 0 },
    { x: 0, y: 1 },
    { x: 0, y: -1 },
  ];
  const offsets = [0.45, 0.75];
  return [
    target,
    ...offsets.flatMap((offset) =>
      directions.map((direction) => ({
        x: target.x + direction.x * offset,
        y: target.y + direction.y * offset,
      })),
    ),
  ];
};

const doorRoutePoints = (door: Door): Vec2[] => {
  const panel = door.panel;
  const offset = 0.95;
  if (panel.halfW >= panel.halfH) {
    return [
      { x: panel.x, y: panel.y },
      { x: panel.x, y: panel.y - offset },
      { x: panel.x, y: panel.y + offset },
    ];
  }
  return [
    { x: panel.x, y: panel.y },
    { x: panel.x - offset, y: panel.y },
    { x: panel.x + offset, y: panel.y },
  ];
};

const routeCandidates = (scenario: Scenario): Vec2[] =>
  dedupePoints([
    ...scenario.doors.flatMap(doorRoutePoints),
    ...scenario.pois.map((poi) => ({ ...poi.position })),
    ...scenario.rooms
      .filter((room) => room.kind === 'corridor' || room.kind === 'connector')
      .map((room) => ({ x: room.x, y: room.y })),
    ...scenario.spawnZones.map((zone) => ({ x: zone.x, y: zone.y })),
  ]);

const routeGraphKey = (scenario: Scenario, obstacles: Rect[], clearanceRadius: number): string =>
  `${scenario.id}:${clearanceRadius.toFixed(2)}:${obstacles.map((rect) => rect.id).join(',')}`;

const quantize = (value: number, scale: number): number => Math.round(value / scale) * scale;

const waypointCacheKey = (
  position: Vec2,
  target: Vec2,
  scenario: Scenario,
  obstacles: Rect[],
  clearanceRadius: number,
): string =>
  [
    routeGraphKey(scenario, obstacles, clearanceRadius),
    quantize(position.x, 0.25).toFixed(2),
    quantize(position.y, 0.25).toFixed(2),
    target.x.toFixed(2),
    target.y.toFixed(2),
  ].join(':');

const routeGraph = (scenario: Scenario, obstacles: Rect[], clearanceRadius: number): RouteGraph => {
  const key = routeGraphKey(scenario, obstacles, clearanceRadius);
  const cached = routeGraphCache.get(key);
  if (cached) return cached;

  const nodes = routeCandidates(scenario).filter((point) => isCircleFree(point, clearanceRadius, obstacles, scenario));
  const edges = nodes.map<number[]>(() => []);
  for (let i = 0; i < nodes.length - 1; i += 1) {
    for (let j = i + 1; j < nodes.length; j += 1) {
      if (!isSegmentFree(nodes[i], nodes[j], clearanceRadius, obstacles, scenario)) continue;
      edges[i].push(j);
      edges[j].push(i);
    }
  }

  const graph = { nodes, edges };
  routeGraphCache.set(key, graph);
  return graph;
};

export const navigationWaypoint = (
  position: Vec2,
  target: Vec2,
  radius: number,
  obstacles: Rect[],
  scenario: Scenario,
  clearanceBuffer = 0.08,
): Vec2 => {
  const clearanceRadius = radius + clearanceBuffer;
  const cacheKey = waypointCacheKey(position, target, scenario, obstacles, clearanceRadius);
  const cached = waypointCache.get(cacheKey);
  if (cached) return cached;

  const goalCandidates = dedupePoints(targetApproachPoints(target)).filter((point) =>
    isCircleFree(point, clearanceRadius, obstacles, scenario),
  );
  const goals = goalCandidates.length > 0 ? goalCandidates : [target];

  for (const goal of goals) {
    if (isSegmentFree(position, goal, clearanceRadius, obstacles, scenario)) {
      waypointCache.set(cacheKey, goal);
      return goal;
    }
  }

  const graph = routeGraph(scenario, obstacles, clearanceRadius);
  const nodes = [position, ...graph.nodes, ...goals];
  const goalStart = 1 + graph.nodes.length;
  const distances = nodes.map(() => Number.POSITIVE_INFINITY);
  const previous = nodes.map<number | undefined>(() => undefined);
  const visited = new Set<number>();
  distances[0] = 0;

  for (let pass = 0; pass < nodes.length; pass += 1) {
    let current = -1;
    let bestDistance = Number.POSITIVE_INFINITY;
    for (let index = 0; index < nodes.length; index += 1) {
      if (!visited.has(index) && distances[index] < bestDistance) {
        current = index;
        bestDistance = distances[index];
      }
    }
    if (current < 0 || current >= goalStart) break;
    visited.add(current);

    const neighbors =
      current === 0
        ? graph.nodes.map((_, index) => index + 1)
        : graph.edges[current - 1].map((index) => index + 1);
    for (const next of [...neighbors, ...goals.map((_, index) => goalStart + index)]) {
      if (next === current || visited.has(next)) continue;
      const edgeIsCachedGraphEdge = current > 0 && current < goalStart && next > 0 && next < goalStart;
      if (!edgeIsCachedGraphEdge && !isSegmentFree(nodes[current], nodes[next], clearanceRadius, obstacles, scenario)) continue;
      const candidateDistance = distances[current] + distance(nodes[current], nodes[next]);
      if (candidateDistance < distances[next]) {
        distances[next] = candidateDistance;
        previous[next] = current;
      }
    }
  }

  let bestGoal = -1;
  for (let index = goalStart; index < nodes.length; index += 1) {
    if (bestGoal < 0 || distances[index] < distances[bestGoal]) bestGoal = index;
  }

  if (bestGoal >= 0 && Number.isFinite(distances[bestGoal])) {
    let cursor = bestGoal;
    while (previous[cursor] !== undefined && previous[cursor] !== 0) cursor = previous[cursor]!;
    waypointCache.set(cacheKey, nodes[cursor]);
    return nodes[cursor];
  }

  let fallback: Vec2 | undefined;
  let fallbackScore = Number.POSITIVE_INFINITY;
  for (const candidate of graph.nodes) {
    if (!isSegmentFree(position, candidate, clearanceRadius, obstacles, scenario)) continue;
    const score = distance(position, candidate) + distance(candidate, target) * 1.8;
    if (score < fallbackScore) {
      fallback = candidate;
      fallbackScore = score;
    }
  }
  const waypoint = fallback ?? target;
  waypointCache.set(cacheKey, waypoint);
  return waypoint;
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
