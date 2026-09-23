import { useState } from "react";
import { HitEventLog } from "./components/HitEventLog";
import { JudgeCameraFeed } from "./components/JudgeCameraFeed";
import { MatchControlPanel } from "./components/MatchControlPanel";
import { MatchHistoryTable } from "./components/MatchHistoryTable";
import { ScoreBoard } from "./components/ScoreBoard";
import { TeamStatusPanel } from "./components/TeamStatusPanel";
import { useRos } from "./lib/ros";

/**
 * 입력: 없음(props X) — 로그인 없이 바로 진입(팀 PC와 달리 게이트 없음).
 * 출력: 없음(최상위) — 스코어보드/카메라/양팀상태/경기제어/명중로그/경기기록 6개 패널을 배치.
 */
export default function App() {
  const { ros, status } = useRos();
  const [historyRefreshKey, setHistoryRefreshKey] = useState(0);

  return (
    <div className="app">
      <header className="banner">
        <span className="banner__team">심판 PC 관제</span>
        <span className={`banner__conn banner__conn--${status}`}>
          {status === "connected" ? "● 연결됨" : status === "connecting" ? "○ 연결중" : "○ 연결 끊김"}
        </span>
      </header>

      <ScoreBoard ros={ros} />

      <main className="grid grid--judge">
        <div className="grid__col">
          <JudgeCameraFeed ros={ros} />
          <div className="team-status-row">
            <TeamStatusPanel ros={ros} color="blue" />
            <TeamStatusPanel ros={ros} color="red" />
          </div>
        </div>
        <div className="grid__side">
          <MatchControlPanel ros={ros} onMatchEnded={() => setHistoryRefreshKey((k) => k + 1)} />
          <HitEventLog ros={ros} />
          <MatchHistoryTable refreshKey={historyRefreshKey} />
        </div>
      </main>
    </div>
  );
}
