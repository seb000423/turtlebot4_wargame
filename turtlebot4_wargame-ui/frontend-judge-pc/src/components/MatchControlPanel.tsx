import { useState } from "react";
import type ROSLIB from "roslib";
import { api } from "../lib/api";
import { useTopic } from "../lib/ros";
import { topics, type StringMsg } from "../lib/topics";

/**
 * 입력: ros(현재 match_state 구독용), onMatchEnded(경기 종료 성공 시 부모에 알려 기록 테이블 갱신 트리거).
 * 출력: 없음 — 버튼 클릭 시 REST POST /api/match/start 또는 /end 호출, 실패하면 에러 문구 표시.
 */
export function MatchControlPanel({ ros, onMatchEnded }: { ros: ROSLIB.Ros; onMatchEnded: () => void }) {
  const matchState = useTopic<StringMsg>(ros, topics.matchState, "std_msgs/String");
  const state = matchState?.data ?? "IDLE";
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleStart = async () => {
    setPending(true);
    setError(null);
    try {
      await api.startMatch();
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setPending(false);
    }
  };

  const handleEnd = async () => {
    setPending(true);
    setError(null);
    try {
      await api.endMatch();
      onMatchEnded();
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setPending(false);
    }
  };

  return (
    <section className="panel match-control">
      <h2>경기 제어</h2>
      <div className="match-control__buttons">
        <button className="btn btn--start" disabled={pending || state === "RUNNING"} onClick={handleStart}>
          경기 시작
        </button>
        <button className="btn btn--end" disabled={pending || state !== "RUNNING"} onClick={handleEnd}>
          경기 종료
        </button>
      </div>
      {error && <p className="error">{error}</p>}
    </section>
  );
}
