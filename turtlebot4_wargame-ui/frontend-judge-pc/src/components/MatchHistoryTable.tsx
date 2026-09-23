import { useEffect, useState } from "react";
import { api, type MatchSummary } from "../lib/api";

/**
 * 입력: refreshKey(값이 바뀔 때마다 재조회 트리거 — MatchControlPanel이 경기 종료 시 증가시킴).
 * 출력: 없음 — REST GET /api/matches 결과를 표로 렌더(WS 아님, REST 폴링 방식).
 */
export function MatchHistoryTable({ refreshKey }: { refreshKey: number }) {
  const [matches, setMatches] = useState<MatchSummary[]>([]);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    api
      .listMatches()
      .then(setMatches)
      .catch((e) => setError((e as Error).message));
  }, [refreshKey]);

  return (
    <section className="panel">
      <h2>경기 기록</h2>
      {error && <p className="error">{error}</p>}
      {matches.length === 0 ? (
        <p className="muted">기록된 경기가 없음</p>
      ) : (
        <table className="history-table">
          <thead>
            <tr>
              <th>#</th>
              <th>시작</th>
              <th>상태</th>
              <th>승자</th>
            </tr>
          </thead>
          <tbody>
            {matches.map((m) => (
              <tr key={m.id}>
                <td>{m.id}</td>
                <td>{new Date(m.started_at).toLocaleString()}</td>
                <td>{m.status}</td>
                <td>{m.winner ? m.winner.toUpperCase() : "-"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </section>
  );
}
