export class SeededRandom {
  private state: number;

  constructor(seed: number) {
    this.state = seed >>> 0 || 1;
  }

  next(): number {
    this.state = (1664525 * this.state + 1013904223) >>> 0;
    return this.state / 0x100000000;
  }

  range(min: number, max: number): number {
    return min + (max - min) * this.next();
  }

  int(min: number, maxInclusive: number): number {
    return Math.floor(this.range(min, maxInclusive + 1));
  }

  pick<T>(items: T[]): T {
    if (items.length === 0) {
      throw new Error('Cannot pick from an empty array.');
    }
    return items[this.int(0, items.length - 1)];
  }

  weighted<T>(items: T[], weight: (item: T) => number): T {
    const total = items.reduce((sum, item) => sum + Math.max(0, weight(item)), 0);
    if (total <= 0) {
      return this.pick(items);
    }
    let cursor = this.range(0, total);
    for (const item of items) {
      cursor -= Math.max(0, weight(item));
      if (cursor <= 0) {
        return item;
      }
    }
    return items[items.length - 1];
  }
}
