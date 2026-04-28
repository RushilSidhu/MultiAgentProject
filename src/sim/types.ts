export type Vec2 = {
  x: number;
  y: number;
};

export type Rect = {
  id: string;
  x: number;
  y: number;
  halfW: number;
  halfH: number;
  kind: 'wall' | 'bed' | 'furniture';
};

export type Door = {
  id: string;
  label: string;
  roomId: string;
  panel: Rect;
  triggerRadius: number;
};

export type DoorState = {
  id: string;
  isOpen: boolean;
};

export type Room = {
  id: string;
  name: string;
  kind: 'patient' | 'corridor' | 'treatment' | 'connector' | 'clinical';
  x: number;
  y: number;
  w: number;
  h: number;
};

export type Poi = {
  id: string;
  label: string;
  roomId: string;
  position: Vec2;
  taskTarget: boolean;
  staffStop: boolean;
};

export type SpawnZone = {
  id: string;
  label: string;
  x: number;
  y: number;
  halfW: number;
  halfH: number;
  weight: number;
  agentKinds: Array<'robot' | 'staff'>;
};

export type TaskClass = {
  id: string;
  label: string;
  priority: number;
  serviceSeconds: number;
  deadlineSeconds: number;
  originPoiIds: string[];
  destinationPoiIds: string[];
};

export type Scenario = {
  id: string;
  name: string;
  width: number;
  height: number;
  rooms: Room[];
  obstacles: Rect[];
  doors: Door[];
  pois: Poi[];
  spawnZones: SpawnZone[];
  taskClasses: TaskClass[];
};

export type RobotStatus = 'idle' | 'returning' | 'assigned' | 'servicing' | 'blocked' | 'down';
export type StaffStatus = 'walking' | 'dwelling';
export type StaffRole = 'floor_nurse' | 'charge_nurse' | 'cart_nurse';
export type StaffActivityState = 'working-in-room' | 'transit' | 'fetch-cart' | 'dwelling';
export type TaskStatus = 'queued' | 'assigned' | 'inService' | 'completed' | 'cancelled';

export type RobotDecisionPhase = 'idle' | 'pickup' | 'delivery' | 'complete';

export type RobotDiagnostics = {
  updatedAt: number;
  action: 'velocity' | 'hold' | 'complete-service';
  phase: RobotDecisionPhase;
  note: string;
  reason?: string;
  taskId?: string;
  taskLabel?: string;
  target?: Vec2;
  targetLabel?: string;
  sensedStaffIds: string[];
  sensedRobotIds: string[];
  staffYieldRadius: number;
  nearMissRadius: number;
  linear?: number;
  angular?: number;
};

export type RobotState = {
  id: string;
  position: Vec2;
  theta: number;
  radius: number;
  maxSpeed: number;
  status: RobotStatus;
  taskId?: string;
  trail: Vec2[];
  diagnostics?: RobotDiagnostics;
};

export type StaffState = {
  id: string;
  position: Vec2;
  theta: number;
  radius: number;
  speed: number;
  status: StaffStatus;
  role: StaffRole;
  state: StaffActivityState;
  hasCart: boolean;
  goal: Vec2;
  goalRoomId?: string;
  workRoomId: string;
  workRemaining: number;
  microMoveRemaining: number;
  intent?: string;
  dwellRemaining: number;
  trail: Vec2[];
};

export type TaskState = {
  id: string;
  classId: string;
  label: string;
  priority: number;
  origin: Poi;
  destination: Poi;
  status: TaskStatus;
  createdAt: number;
  assignedRobotId?: string;
  serviceStartedAt?: number;
  completedAt?: number;
  deadlineAt: number;
};

export type SimulationEvent =
  | { time: number; kind: 'task-created'; taskId: string; label: string }
  | { time: number; kind: 'task-assigned'; taskId: string; robotId: string }
  | { time: number; kind: 'task-completed'; taskId: string; robotId: string; elapsed: number }
  | { time: number; kind: 'collision'; a: string; b: string; severity: 'collision' | 'near-miss' }
  | { time: number; kind: 'robot-blocked'; robotId: string; reason: string };

export type CollisionBreakdown = {
  robotStaff: number;
  robotRobot: number;
  robotWall: number;
};

export type SimulationScoreBreakdown = {
  score: number;
  taskCost: number;
  wallPenalty: number;
  robotPenalty: number;
  humanMultiplier: number;
};

export type Metrics = {
  completedTasks: number;
  queuedTasks: number;
  activeTasks: number;
  collisions: number;
  collisionBreakdown: CollisionBreakdown;
  nearMisses: number;
  averageCompletionSeconds: number;
  robotUtilization: number;
  completionTimes: number[];
  uncompletedTaskCreatedAt: number[];
  score: SimulationScoreBreakdown;
};

export type SimulationSnapshot = {
  time: number;
  seed: number;
  scenario: Scenario;
  doorStates: DoorState[];
  robots: RobotState[];
  staff: StaffState[];
  tasks: TaskState[];
  events: SimulationEvent[];
  metrics: Metrics;
};
