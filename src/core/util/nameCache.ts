// implement singleton pattern for name cache
export class NameCache {
  private static instance: NameCache;
  private cache: Map<string, string[]>;

  private constructor() {
    this.cache = new Map<string, string[]>();
  }

  public static getInstance(): NameCache {
    if (!NameCache.instance) {
      NameCache.instance = new NameCache();
    }
    return NameCache.instance;
  }

  public getNames(key: string): string[] | undefined {
    return this.cache.get(key);
  }
  public addName(key: string, names: string): void {
    if (!this.cache.has(key)) {
      this.cache.set(key, []);
    }
    const existingNames = this.cache.get(key);
    if (existingNames) {
      existingNames.push(names);
    }
    this.cache.set(key, existingNames || []);
  }
  public clearCache(): void {
    this.cache.clear();
  }
}