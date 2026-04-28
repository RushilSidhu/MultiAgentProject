import { useEffect, useRef, type MouseEvent } from 'react';
import type { Rect, RobotState, Room, SimulationSnapshot, TaskState, Vec2 } from '../sim/types';

export type CanvasLayerSettings = {
  grid: boolean;
  trails: boolean;
  sensing: boolean;
  labels: boolean;
  routes: boolean;
  doors: boolean;
};

type Props = {
  snapshot: SimulationSnapshot;
  selectedTime?: number;
  selectedRobotId?: string;
  layers?: Partial<CanvasLayerSettings>;
  onSelectRobot?: (robotId: string) => void;
};

const defaultLayers: CanvasLayerSettings = {
  grid: true,
  trails: true,
  sensing: true,
  labels: true,
  routes: true,
  doors: true,
};

const roomColor: Record<Room['kind'], string> = {
  patient: '#172b45',
  corridor: '#171a24',
  treatment: '#153542',
  connector: '#202737',
  clinical: '#27304a',
};

const obstacleColor: Record<Rect['kind'], string> = {
  wall: '#777b86',
  bed: '#d8deed',
  furniture: '#595f70',
};

const statusColor: Record<string, string> = {
  idle: '#f3d65b',
  assigned: '#5be78d',
  servicing: '#77b9ff',
  blocked: '#ff9f43',
  down: '#666b75',
};

const STAFF_VISUAL_RADIUS = 0.62;
const CART_NURSE_VISUAL_RADIUS = 0.48;
const LIDAR_MAX_RANGE = 6;
const LIDAR_FOV = (120 * Math.PI) / 180;
const LIDAR_RAY_COUNT = 25;
const LIDAR_STEP = 0.15;

export function CanvasRenderer({ snapshot, selectedTime, selectedRobotId, layers: layerOverrides, onSelectRobot }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const layers = { ...defaultLayers, ...layerOverrides };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const pixelRatio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth * pixelRatio;
    const height = canvas.clientHeight * pixelRatio;
    canvas.width = width;
    canvas.height = height;
    ctx.scale(pixelRatio, pixelRatio);
    draw(ctx, canvas.clientWidth, canvas.clientHeight, snapshot, selectedTime, selectedRobotId, layers);
  }, [snapshot, selectedTime, selectedRobotId, layerOverrides]);

  const handleClick = (event: MouseEvent<HTMLCanvasElement>) => {
    if (!onSelectRobot) return;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const world = screenToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.clientWidth, canvas.clientHeight, snapshot);
    const hit = snapshot.robots.find((robot) => distance(robot.position, world) <= robot.radius * 2.1);
    if (hit) onSelectRobot(hit.id);
  };

  return (
    <canvas
      ref={canvasRef}
      className="sim-canvas"
      aria-label="Hospital multi-agent simulation"
      onClick={handleClick}
    />
  );
}

function draw(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  snapshot: SimulationSnapshot,
  selectedTime?: number,
  selectedRobotId?: string,
  layers: CanvasLayerSettings = defaultLayers,
) {
  const { scenario } = snapshot;
  const scale = Math.min(width / scenario.width, height / scenario.height);
  const offsetX = (width - scenario.width * scale) / 2;
  const offsetY = (height - scenario.height * scale) / 2;
  const tx = (x: number) => offsetX + x * scale;
  const ty = (y: number) => offsetY + (scenario.height - y) * scale;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#090b12';
  ctx.fillRect(0, 0, width, height);

  for (const room of scenario.rooms) {
    ctx.fillStyle = roomColor[room.kind];
    ctx.globalAlpha = room.kind === 'corridor' || room.kind === 'connector' ? 0.68 : 0.92;
    ctx.fillRect(tx(room.x - room.w / 2), ty(room.y + room.h / 2), room.w * scale, room.h * scale);
    ctx.globalAlpha = 1;
    ctx.fillStyle = '#b7c7df';
    ctx.font = `${Math.max(9, scale * 0.42)}px Inter, system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.fillText(room.name, tx(room.x), ty(room.y));
  }

  if (layers.grid) {
    drawGrid(ctx, scenario.width, scenario.height, scale, tx, ty);
  }

  for (const obstacle of scenario.obstacles) {
    drawObstacle(ctx, obstacle, scale, tx, ty);
  }

  if (layers.doors) {
    for (const door of scenario.doors) {
      const isOpen = snapshot.doorStates.find((state) => state.id === door.id)?.isOpen ?? false;
      drawDoor(ctx, door.panel, isOpen, scale, tx, ty);
    }
  }

  for (const task of snapshot.tasks.filter((task) => task.status !== 'completed')) {
    drawTask(ctx, task, scale, tx, ty);
  }

  if (layers.routes) {
    for (const robot of snapshot.robots) {
      if (robot.diagnostics?.target) {
        drawRoute(ctx, robot.position, robot.diagnostics.target, statusColor[robot.status] ?? '#ffffff', scale, tx, ty);
      }
    }
  }

  if (layers.trails) {
    for (const person of snapshot.staff) {
      drawTrail(ctx, person.trail, '#39a7ff', scale, tx, ty);
    }

    for (const robot of snapshot.robots) {
      drawTrail(ctx, robot.trail, statusColor[robot.status] ?? '#ffffff', scale, tx, ty);
    }
  }

  if (layers.sensing) {
    for (const robot of snapshot.robots) {
      if (!selectedRobotId || robot.id === selectedRobotId) {
        drawSensingOverlay(ctx, robot, snapshot, scale, tx, ty);
        drawLidar(ctx, robot.position, robot.theta, snapshot, scale, tx, ty);
      }
    }
  }

  for (const person of snapshot.staff) {
    const color = person.hasCart ? '#236ccf' : '#22c4df';
    const staffBodyRadius = person.hasCart ? CART_NURSE_VISUAL_RADIUS : STAFF_VISUAL_RADIUS;
    drawStaffCircle(ctx, person.position, staffBodyRadius, color, scale, tx, ty);
    if (person.hasCart) {
      drawCartAttachment(ctx, person.position, staffBodyRadius, person.theta, color, scale, tx, ty);
    }
  }

  for (const robot of snapshot.robots) {
    drawRobotSquare(
      ctx,
      robot.position,
      robot.radius,
      robot.theta,
      statusColor[robot.status] ?? '#ffffff',
      robot.id === selectedRobotId,
      scale,
      tx,
      ty,
    );
  }

  for (const person of snapshot.staff) {
    if (person.status === 'walking') {
      const staffBodyRadius = person.hasCart ? CART_NURSE_VISUAL_RADIUS : STAFF_VISUAL_RADIUS;
      drawDirectionArrow(ctx, person.position, staffBodyRadius, person.theta, '#dff3ff', scale, tx, ty);
    }
  }

  for (const robot of snapshot.robots) {
    if (robot.status !== 'idle' && robot.status !== 'down' && robot.status !== 'blocked') {
      drawDirectionArrow(ctx, robot.position, robot.radius, robot.theta, '#ffffff', scale, tx, ty);
    }
  }

  if (layers.labels) {
    for (const person of snapshot.staff) {
      const staffBodyRadius = person.hasCart ? CART_NURSE_VISUAL_RADIUS : STAFF_VISUAL_RADIUS;
      drawAgentLabel(ctx, person.position, staffBodyRadius, person.id, scale, tx, ty);
    }

    for (const robot of snapshot.robots) {
      drawAgentLabel(ctx, robot.position, robot.radius, robot.id, scale, tx, ty);
    }
  }

  ctx.fillStyle = '#d9e7ff';
  ctx.font = '13px ui-monospace, SFMono-Regular, Consolas, monospace';
  ctx.textAlign = 'left';
  ctx.fillText(`T=${snapshot.time.toFixed(1)}s | seed=${snapshot.seed}`, 14, 22);
  if (selectedTime !== undefined) {
    ctx.fillText(`Replay cursor=${selectedTime.toFixed(1)}s`, 14, 42);
  }
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  worldW: number,
  worldH: number,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = 1;
  for (let x = 0; x <= worldW; x += 5) {
    ctx.beginPath();
    ctx.moveTo(tx(x), ty(0));
    ctx.lineTo(tx(x), ty(worldH));
    ctx.stroke();
  }
  for (let y = 0; y <= worldH; y += 5) {
    ctx.beginPath();
    ctx.moveTo(tx(0), ty(y));
    ctx.lineTo(tx(worldW), ty(y));
    ctx.stroke();
  }
}

function drawObstacle(
  ctx: CanvasRenderingContext2D,
  obstacle: Rect,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  ctx.fillStyle = obstacleColor[obstacle.kind];
  ctx.strokeStyle = obstacle.kind === 'wall' ? '#aeb2be' : '#82899a';
  ctx.lineWidth = 0.8;
  ctx.fillRect(
    tx(obstacle.x - obstacle.halfW),
    ty(obstacle.y + obstacle.halfH),
    obstacle.halfW * 2 * scale,
    obstacle.halfH * 2 * scale,
  );
  ctx.strokeRect(
    tx(obstacle.x - obstacle.halfW),
    ty(obstacle.y + obstacle.halfH),
    obstacle.halfW * 2 * scale,
    obstacle.halfH * 2 * scale,
  );
}

function drawDoor(
  ctx: CanvasRenderingContext2D,
  panel: Rect,
  isOpen: boolean,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  ctx.save();
  ctx.globalAlpha = isOpen ? 0.35 : 1;
  ctx.fillStyle = isOpen ? '#5be78d' : '#7dd3fc';
  ctx.strokeStyle = isOpen ? '#a7f3d0' : '#e0f2fe';
  ctx.lineWidth = Math.max(1, scale * 0.04);
  ctx.fillRect(tx(panel.x - panel.halfW), ty(panel.y + panel.halfH), panel.halfW * 2 * scale, panel.halfH * 2 * scale);
  ctx.strokeRect(tx(panel.x - panel.halfW), ty(panel.y + panel.halfH), panel.halfW * 2 * scale, panel.halfH * 2 * scale);
  ctx.restore();
}

function drawTrail(
  ctx: CanvasRenderingContext2D,
  trail: Vec2[],
  color: string,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  if (trail.length < 2) return;
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.26;
  ctx.lineWidth = Math.max(1, scale * 0.08);
  ctx.beginPath();
  ctx.moveTo(tx(trail[0].x), ty(trail[0].y));
  for (const point of trail.slice(1)) {
    ctx.lineTo(tx(point.x), ty(point.y));
  }
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawRoute(
  ctx: CanvasRenderingContext2D,
  from: Vec2,
  to: Vec2,
  color: string,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.42;
  ctx.setLineDash([Math.max(5, scale * 0.28), Math.max(4, scale * 0.18)]);
  ctx.lineWidth = Math.max(1.5, scale * 0.08);
  ctx.beginPath();
  ctx.moveTo(tx(from.x), ty(from.y));
  ctx.lineTo(tx(to.x), ty(to.y));
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = color;
  ctx.globalAlpha = 0.75;
  ctx.beginPath();
  ctx.arc(tx(to.x), ty(to.y), Math.max(4, scale * 0.18), 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawSensingOverlay(
  ctx: CanvasRenderingContext2D,
  robot: RobotState,
  snapshot: SimulationSnapshot,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  const diagnostics = robot.diagnostics;
  if (!diagnostics) return;

  ctx.save();
  ctx.strokeStyle = 'rgba(142, 255, 192, 0.22)';
  ctx.fillStyle = 'rgba(142, 255, 192, 0.035)';
  ctx.lineWidth = Math.max(1, scale * 0.035);
  ctx.beginPath();
  ctx.arc(tx(robot.position.x), ty(robot.position.y), 5 * scale, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = 'rgba(255, 209, 102, 0.34)';
  ctx.beginPath();
  ctx.arc(tx(robot.position.x), ty(robot.position.y), diagnostics.staffYieldRadius * scale, 0, Math.PI * 2);
  ctx.stroke();

  const sensedStaff = new Set(diagnostics.sensedStaffIds);
  for (const person of snapshot.staff) {
    if (!sensedStaff.has(person.id)) continue;
    ctx.strokeStyle = 'rgba(255, 209, 102, 0.7)';
    ctx.lineWidth = Math.max(1, scale * 0.05);
    ctx.beginPath();
    ctx.moveTo(tx(robot.position.x), ty(robot.position.y));
    ctx.lineTo(tx(person.position.x), ty(person.position.y));
    ctx.stroke();
  }
  ctx.restore();
}

function drawStaffCircle(
  ctx: CanvasRenderingContext2D,
  position: Vec2,
  radius: number,
  color: string,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  ctx.fillStyle = color;
  ctx.strokeStyle = '#0b0e14';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(tx(position.x), ty(position.y), radius * scale, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
}

function drawCartAttachment(
  ctx: CanvasRenderingContext2D,
  position: Vec2,
  radius: number,
  theta: number,
  color: string,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  // Carts are intentionally larger than nurse body circles and lead in heading direction.
  const cartW = radius * 1.95;
  const cartH = radius * 1.3;
  const lead = {
    x: position.x + Math.cos(theta) * radius * 1.5,
    y: position.y + Math.sin(theta) * radius * 1.5,
  };

  ctx.save();
  ctx.translate(tx(lead.x), ty(lead.y));
  ctx.rotate(-theta);
  ctx.fillStyle = color;
  ctx.strokeStyle = '#0b0e14';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.rect((-cartW * scale) / 2, (-cartH * scale) / 2, cartW * scale, cartH * scale);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawRobotSquare(
  ctx: CanvasRenderingContext2D,
  position: Vec2,
  radius: number,
  theta: number,
  color: string,
  selected: boolean,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  const side = radius * 2;
  ctx.save();
  ctx.translate(tx(position.x), ty(position.y));
  ctx.rotate(-theta);
  if (selected) {
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(0, 0, radius * 1.75 * scale, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.fillStyle = color;
  ctx.strokeStyle = '#0b0e14';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.rect((-side * scale) / 2, (-side * scale) / 2, side * scale, side * scale);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawDirectionArrow(
  ctx: CanvasRenderingContext2D,
  position: Vec2,
  radius: number,
  theta: number,
  color: string,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  const start = {
    x: position.x + Math.cos(theta) * radius * 0.2,
    y: position.y + Math.sin(theta) * radius * 0.2,
  };
  const end = {
    x: position.x + Math.cos(theta) * radius * 2.2,
    y: position.y + Math.sin(theta) * radius * 2.2,
  };
  const headSize = radius * 0.5;
  const left = {
    x: end.x - Math.cos(theta - Math.PI / 7) * headSize,
    y: end.y - Math.sin(theta - Math.PI / 7) * headSize,
  };
  const right = {
    x: end.x - Math.cos(theta + Math.PI / 7) * headSize,
    y: end.y - Math.sin(theta + Math.PI / 7) * headSize,
  };

  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = Math.max(1, scale * 0.08);
  ctx.globalAlpha = 0.9;
  ctx.beginPath();
  ctx.moveTo(tx(start.x), ty(start.y));
  ctx.lineTo(tx(end.x), ty(end.y));
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(tx(end.x), ty(end.y));
  ctx.lineTo(tx(left.x), ty(left.y));
  ctx.lineTo(tx(right.x), ty(right.y));
  ctx.closePath();
  ctx.fill();
  ctx.globalAlpha = 1;
}

function drawAgentLabel(
  ctx: CanvasRenderingContext2D,
  position: Vec2,
  radius: number,
  label: string,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  ctx.fillStyle = '#f7fbff';
  ctx.font = `${Math.max(10, scale * 0.35)}px Inter, system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.fillText(label, tx(position.x), ty(position.y - radius - 0.35));
}

function drawLidar(
  ctx: CanvasRenderingContext2D,
  position: Vec2,
  theta: number,
  snapshot: SimulationSnapshot,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  const halfFov = LIDAR_FOV / 2;
  const obstacles = visibleObstacles(snapshot);
  ctx.strokeStyle = 'rgba(142, 255, 192, 0.34)';
  ctx.lineWidth = Math.max(1, scale * 0.04);
  for (let i = 0; i < LIDAR_RAY_COUNT; i += 1) {
    const blend = i / (LIDAR_RAY_COUNT - 1);
    const rayTheta = theta - halfFov + blend * LIDAR_FOV;
    const hit = castRay(position, rayTheta, snapshot.scenario, obstacles);
    ctx.beginPath();
    ctx.moveTo(tx(position.x), ty(position.y));
    ctx.lineTo(tx(hit.x), ty(hit.y));
    ctx.stroke();
  }
}

function visibleObstacles(snapshot: SimulationSnapshot): Rect[] {
  return snapshot.scenario.obstacles;
}

function castRay(
  origin: Vec2,
  rayTheta: number,
  scenario: SimulationSnapshot['scenario'],
  obstacles: Rect[],
): Vec2 {
  const dirX = Math.cos(rayTheta);
  const dirY = Math.sin(rayTheta);
  for (let d = 0; d <= LIDAR_MAX_RANGE; d += LIDAR_STEP) {
    const probe = {
      x: origin.x + dirX * d,
      y: origin.y + dirY * d,
    };
    const outOfBounds = probe.x <= 0 || probe.y <= 0 || probe.x >= scenario.width || probe.y >= scenario.height;
    if (outOfBounds) return probe;
    const hitObstacle = obstacles.some(
      (rect) =>
        probe.x >= rect.x - rect.halfW &&
        probe.x <= rect.x + rect.halfW &&
        probe.y >= rect.y - rect.halfH &&
        probe.y <= rect.y + rect.halfH,
    );
    if (hitObstacle) return probe;
  }
  return {
    x: origin.x + dirX * LIDAR_MAX_RANGE,
    y: origin.y + dirY * LIDAR_MAX_RANGE,
  };
}

function drawTask(
  ctx: CanvasRenderingContext2D,
  task: TaskState,
  scale: number,
  tx: (value: number) => number,
  ty: (value: number) => number,
) {
  const pos = task.status === 'queued' || task.status === 'assigned' ? task.origin.position : task.destination.position;
  ctx.fillStyle = task.priority >= 4 ? '#ff5964' : '#ffd166';
  ctx.strokeStyle = '#fff3bd';
  ctx.lineWidth = 1.5;
  const x = tx(pos.x);
  const y = ty(pos.y);
  const r = Math.max(5, scale * 0.25);
  ctx.beginPath();
  for (let i = 0; i < 5; i += 1) {
    const angle = -Math.PI / 2 + (i * Math.PI * 2) / 5;
    const px = x + Math.cos(angle) * r;
    const py = y + Math.sin(angle) * r;
    if (i === 0) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  }
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
}

function screenToWorld(screenX: number, screenY: number, width: number, height: number, snapshot: SimulationSnapshot): Vec2 {
  const { scenario } = snapshot;
  const scale = Math.min(width / scenario.width, height / scenario.height);
  const offsetX = (width - scenario.width * scale) / 2;
  const offsetY = (height - scenario.height * scale) / 2;
  return {
    x: (screenX - offsetX) / scale,
    y: scenario.height - (screenY - offsetY) / scale,
  };
}

function distance(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}
