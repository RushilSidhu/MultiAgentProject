import type { Door, Poi, Rect, Room, Scenario, SpawnZone, TaskClass } from '../sim/types';

const room = (
  id: string,
  name: string,
  kind: Room['kind'],
  x: number,
  y: number,
  w: number,
  h: number,
): Room => ({ id, name, kind, x, y, w, h });

const obstacle = (
  id: string,
  kind: Rect['kind'],
  x: number,
  y: number,
  halfW: number,
  halfH: number,
): Rect => ({ id, kind, x, y, halfW, halfH });

const horizontalWallSegments = (
  idPrefix: string,
  y: number,
  xMin: number,
  xMax: number,
  openings: Array<[number, number]>,
  halfH: number,
): Rect[] => {
  const sorted = [...openings].sort((a, b) => a[0] - b[0]);
  const segments: Rect[] = [];
  let cursor = xMin;
  let idx = 1;
  for (const [openL, openR] of sorted) {
    if (openL > cursor) {
      const segL = cursor;
      const segR = Math.min(openL, xMax);
      if (segR > segL) {
        segments.push(obstacle(`${idPrefix}-${idx++}`, 'wall', (segL + segR) / 2, y, (segR - segL) / 2, halfH));
      }
    }
    cursor = Math.max(cursor, openR);
    if (cursor >= xMax) break;
  }
  if (cursor < xMax) {
    segments.push(obstacle(`${idPrefix}-${idx++}`, 'wall', (cursor + xMax) / 2, y, (xMax - cursor) / 2, halfH));
  }
  return segments;
};

const door = (id: string, label: string, roomId: string, x: number, y: number, halfW: number, halfH: number): Door => ({
  id,
  label,
  roomId,
  panel: obstacle(`${id}-panel`, 'wall', x, y, halfW, halfH),
  triggerRadius: 1.8,
});

const poi = (
  id: string,
  label: string,
  roomId: string,
  x: number,
  y: number,
  taskTarget = true,
  staffStop = true,
): Poi => ({ id, label, roomId, position: { x, y }, taskTarget, staffStop });

const spawnZone = (
  id: string,
  label: string,
  x: number,
  y: number,
  halfW: number,
  halfH: number,
  weight: number,
  agentKinds: SpawnZone['agentKinds'],
): SpawnZone => ({ id, label, x, y, halfW, halfH, weight, agentKinds });

const taskClass = (
  id: string,
  label: string,
  priority: number,
  serviceSeconds: number,
  deadlineSeconds: number,
  originPoiIds: string[],
  destinationPoiIds: string[],
): TaskClass => ({
  id,
  label,
  priority,
  serviceSeconds,
  deadlineSeconds,
  originPoiIds,
  destinationPoiIds,
});

const borderObstacles = [
  obstacle('wall-south', 'wall', 30, 0.125, 30, 0.125),
  obstacle('wall-north', 'wall', 30, 39.875, 30, 0.125),
  obstacle('wall-west', 'wall', 0.125, 20, 0.125, 20),
  obstacle('wall-east', 'wall', 59.875, 20, 0.125, 20),
];

const interiorWalls = [
  obstacle('divider-p101-p102', 'wall', 10, 36.375, 0.15, 3.375),
  obstacle('divider-p102-p103', 'wall', 20, 36.375, 0.15, 3.375),
  obstacle('divider-p103-p104', 'wall', 30, 36.375, 0.15, 3.375),
  obstacle('divider-p104-p105', 'wall', 40, 36.375, 0.15, 3.375),
  obstacle('divider-p105-p106', 'wall', 50, 36.375, 0.15, 3.375),
  obstacle('connector-west', 'wall', 27.85, 25.5, 0.15, 3.5),
  obstacle('connector-east', 'wall', 32.15, 25.5, 0.15, 3.5),
  obstacle('row-c-divider-1', 'wall', 10, 14, 0.15, 4),
  obstacle('row-c-divider-2', 'wall', 20, 14, 0.15, 4),
  obstacle('row-c-divider-3', 'wall', 30, 14, 0.15, 4),
  obstacle('row-c-divider-4', 'wall', 40, 14, 0.15, 4),
  obstacle('row-c-divider-5', 'wall', 50, 14, 0.15, 4),
  ...horizontalWallSegments(
    'patient-hall-wall',
    33,
    0.25,
    59.75,
    [
      [4.25, 5.75],
      [14.25, 15.75],
      [24.25, 25.75],
      [34.25, 35.75],
      [44.25, 45.75],
      [54.25, 55.75],
    ],
    0.15,
  ),
  ...horizontalWallSegments(
    'treatment-hall-wall',
    29,
    0.25,
    59.75,
    [
      [4.125, 5.625],
      [13.375, 14.875],
      [22.625, 24.125],
      [28.0, 32.0],
      [35.875, 37.375],
      [45.125, 46.625],
      [54.375, 55.875],
    ],
    0.15,
  ),
  ...horizontalWallSegments('treatment-lower-wall', 22, 0.25, 59.75, [[28.0, 32.0]], 0.15),
  ...horizontalWallSegments(
    'clinical-hall-wall',
    18,
    0.25,
    59.75,
    [
      [4.25, 5.75],
      [14.25, 15.75],
      [24.25, 25.75],
      [34.25, 35.75],
      [44.25, 45.75],
      [54.25, 55.75],
    ],
    0.15,
  ),
];

const furniture = [
  obstacle('bed-p101', 'bed', 5, 38.5, 1.2, 0.5),
  obstacle('bed-p102', 'bed', 15, 38.5, 1.2, 0.5),
  obstacle('bed-p103', 'bed', 25, 38.5, 1.2, 0.5),
  obstacle('bed-p104', 'bed', 35, 38.5, 1.2, 0.5),
  obstacle('bed-p105', 'bed', 45, 38.5, 1.2, 0.5),
  obstacle('bed-p106', 'bed', 55, 38.5, 1.2, 0.5),
  obstacle('lab-bench', 'furniture', 5, 27, 1.5, 0.5),
  obstacle('rad-table', 'furniture', 15, 24.5, 1, 0.8),
  obstacle('or-table', 'bed', 35, 24.5, 1, 0.6),
  obstacle('exam-table', 'bed', 55, 24.5, 1, 0.6),
  obstacle('pharmacy-shelf', 'furniture', 15, 11.5, 2.5, 0.5),
  obstacle('supply-shelf', 'furniture', 25, 11.5, 2.5, 0.5),
  obstacle('er-bay', 'bed', 55, 13.5, 1, 0.6),
];

const patientRooms = ['P101', 'P102', 'P103', 'P104', 'P105', 'P106'].map((name, index) =>
  room(name.toLowerCase(), name, 'patient', 5 + index * 10, 36.375, 9.6, 6.75),
);

const slidingDoors = [
  door('door-p101', 'P101 Door', 'p101', 5, 33, 0.75, 0.15),
  door('door-p102', 'P102 Door', 'p102', 15, 33, 0.75, 0.15),
  door('door-p103', 'P103 Door', 'p103', 25, 33, 0.75, 0.15),
  door('door-p104', 'P104 Door', 'p104', 35, 33, 0.75, 0.15),
  door('door-p105', 'P105 Door', 'p105', 45, 33, 0.75, 0.15),
  door('door-p106', 'P106 Door', 'p106', 55, 33, 0.75, 0.15),
  door('door-lab', 'LAB Door', 'lab', 4.875, 29, 0.75, 0.15),
  door('door-rad', 'RAD Door', 'rad', 14.125, 29, 0.75, 0.15),
  door('door-pt', 'PT Door', 'pt', 23.375, 29, 0.75, 0.15),
  door('door-or', 'OR Door', 'or', 36.625, 29, 0.75, 0.15),
  door('door-recovery', 'Recovery Door', 'recovery', 45.875, 29, 0.75, 0.15),
  door('door-exam', 'EXAM Door', 'exam', 55.125, 29, 0.75, 0.15),
  door('door-nurse', 'NURSE Door', 'nurse', 5, 18, 0.75, 0.15),
  door('door-pharmacy', 'PHARMACY Door', 'pharmacy', 15, 18, 0.75, 0.15),
  door('door-supply', 'SUPPLY Door', 'supply', 25, 18, 0.75, 0.15),
  door('door-icu-a', 'ICU-A Door', 'icu-a', 35, 18, 0.75, 0.15),
  door('door-icu-b', 'ICU-B Door', 'icu-b', 45, 18, 0.75, 0.15),
  door('door-er', 'ER Door', 'er', 55, 18, 0.75, 0.15),
];

export const hospitalScenario: Scenario = {
  id: 'visual-ward-v1',
  name: 'Visual Ward V1',
  width: 60,
  height: 40,
  rooms: [
    ...patientRooms,
    room('hallway-1', 'Hallway 1', 'corridor', 30, 31, 59.5, 4),
    room('lab', 'LAB', 'treatment', 5, 25.5, 9.1, 7),
    room('rad', 'RAD', 'treatment', 15, 25.5, 8.9, 7),
    room('pt', 'PT', 'treatment', 24, 25.5, 8.9, 7),
    room('connector', 'Connector', 'connector', 30, 25.5, 3.7, 7),
    room('or', 'OR', 'treatment', 36.6, 25.5, 8.9, 7),
    room('recovery', 'Recovery', 'treatment', 46, 25.5, 8.9, 7),
    room('exam', 'EXAM', 'treatment', 55, 25.5, 9.1, 7),
    room('hallway-2', 'Hallway 2', 'corridor', 30, 20, 59.5, 4),
    room('nurse', 'NURSE', 'clinical', 5, 14, 9.6, 8),
    room('pharmacy', 'PHARMACY', 'clinical', 15, 14, 9.6, 8),
    room('supply', 'SUPPLY', 'clinical', 25, 14, 9.6, 8),
    room('icu-a', 'ICU-A', 'clinical', 35, 14, 9.6, 8),
    room('icu-b', 'ICU-B', 'clinical', 45, 14, 9.6, 8),
    room('er', 'ER', 'clinical', 55, 14, 9.6, 8),
  ],
  obstacles: [...borderObstacles, ...interiorWalls, ...furniture],
  doors: slidingDoors,
  pois: [
    poi('bedside-p101', 'P101 Bedside', 'p101', 5, 36),
    poi('bedside-p102', 'P102 Bedside', 'p102', 15, 36),
    poi('bedside-p103', 'P103 Bedside', 'p103', 25, 36),
    poi('bedside-p104', 'P104 Bedside', 'p104', 35, 36),
    poi('bedside-p105', 'P105 Bedside', 'p105', 45, 36),
    poi('bedside-p106', 'P106 Bedside', 'p106', 55, 36),
    poi('lab-pickup', 'Lab Pickup', 'lab', 5, 26),
    poi('rad-table', 'Radiology Table', 'rad', 15, 25.5),
    poi('or-table', 'OR Table', 'or', 35, 25.5),
    poi('exam-table', 'Exam Table', 'exam', 55, 25.5),
    poi('pharmacy-pickup', 'Pharmacy', 'pharmacy', 15, 12.5),
    poi('supply-pickup', 'Supply', 'supply', 25, 12.5),
    poi('icu-a-bed', 'ICU-A Bed', 'icu-a', 35, 15.5),
    poi('icu-b-bed', 'ICU-B Bed', 'icu-b', 45, 15.5),
    poi('er-bay', 'ER Bay', 'er', 55, 12.5),
    poi('nurse-desk', 'Nurse Desk', 'nurse', 5, 12.5, false),
    poi('hallway-1-mid', 'Hallway 1 Mid', 'hallway-1', 30, 31, false),
    poi('hallway-2-mid', 'Hallway 2 Mid', 'hallway-2', 30, 20, false),
  ],
  spawnZones: [
    spawnZone('robot-hallway-2', 'Robot Staging: Hallway 2', 30, 20, 28, 1.5, 2, ['robot']),
    spawnZone('robot-hallway-1', 'Robot Staging: Hallway 1', 30, 31, 28, 1.5, 1, ['robot']),
    spawnZone('staff-hallway-1', 'Staff Traffic: Hallway 1', 30, 31, 28, 1.6, 3, ['staff']),
    spawnZone('staff-hallway-2', 'Staff Traffic: Hallway 2', 30, 20, 28, 1.6, 2, ['staff']),
  ],
  taskClasses: [
    taskClass(
      'med-delivery',
      'Medication Delivery',
      3,
      3,
      90,
      ['pharmacy-pickup'],
      ['bedside-p101', 'bedside-p102', 'bedside-p103', 'bedside-p104', 'bedside-p105', 'bedside-p106', 'icu-a-bed', 'icu-b-bed'],
    ),
    taskClass(
      'supply-run',
      'Supply Run',
      2,
      2,
      120,
      ['supply-pickup'],
      ['exam-table', 'or-table', 'er-bay', 'bedside-p101', 'bedside-p106'],
    ),
    taskClass(
      'specimen-transfer',
      'Specimen Transfer',
      4,
      2,
      75,
      ['bedside-p101', 'bedside-p102', 'bedside-p103', 'bedside-p104', 'bedside-p105', 'bedside-p106', 'er-bay'],
      ['lab-pickup'],
    ),
  ],
};
