import { hexSeed } from "./rand";

export function is9wordString(str: string): boolean {
    if (str.length !== 9) return false;
    let isOk = true;
    str.split("").forEach((s) => {
        if (!hexSeed.includes(s)) isOk = false;
    });
    return isOk;
}