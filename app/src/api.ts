import type { ApiResult } from "./types";

export type Notify = (message: string) => void;

// Every mutating call reports failures to the operator instead of failing silently.
export function makeApi(notify: Notify) {
  async function call<T>(method: "GET" | "POST", route: string, body?: unknown): Promise<ApiResult<T>> {
    const result = await window.haggatrons.api<T>(method, route, body);
    if (!result.ok && method === "POST") notify(result.error || `Request failed (${result.status})`);
    return result;
  }
  return {
    get: <T>(route: string) => call<T>("GET", route),
    post: <T>(route: string, body?: unknown) => call<T>("POST", route, body),
  };
}

export type Api = ReturnType<typeof makeApi>;
