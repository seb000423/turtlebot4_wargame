const API_URL = import.meta.env.VITE_API_URL ?? "http://localhost:8000";

export interface MatchSummary {
  id: number;
  started_at: string;
  ended_at: string | null;
  status: string;
  winner: string | null;
}

export interface HitEventRecord {
  id: number;
  timestamp: string;
  victim_color: string;
  blue_hp_after: number;
  red_hp_after: number;
}

export interface MatchDetail extends MatchSummary {
  hits: HitEventRecord[];
}

async function request<T>(path: string, options?: RequestInit): Promise<T> {
  const res = await fetch(`${API_URL}${path}`, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  if (!res.ok) {
    const detail = await res.json().catch(() => ({}));
    throw new Error(detail.detail ?? `요청 실패: ${res.status}`);
  }
  return res.json() as Promise<T>;
}

/**
 * 백엔드 REST 클라이언트 — 실시간 값(WS)이 아니라 "조회/제어" 성격의 호출만 모아둠.
 * listMatches: 입력 없음 / 출력 MatchSummary[].
 * getMatch: 입력 id / 출력 MatchDetail(hits 포함).
 * startMatch: 입력 없음 / 출력 {match_id, match_state}.
 * endMatch: 입력 없음 / 출력 {match_id, match_state, winner}.
 */
export const api = {
  listMatches: () => request<MatchSummary[]>("/api/matches"),
  getMatch: (id: number) => request<MatchDetail>(`/api/matches/${id}`),
  startMatch: () => request<{ match_id: number; match_state: string }>("/api/match/start", { method: "POST" }),
  endMatch: () =>
    request<{ match_id: number; match_state: string; winner: string | null }>("/api/match/end", { method: "POST" }),
};
