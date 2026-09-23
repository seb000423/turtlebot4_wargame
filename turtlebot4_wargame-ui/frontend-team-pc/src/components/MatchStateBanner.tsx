import type ROSLIB from "roslib";
import type { ConnectionStatus } from "../lib/ros";
import { useTopic } from "../lib/ros";
import { topics, type StringMsg, type TeamColor } from "../lib/topics";

const STATE_LABEL: Record<string, string> = {
  IDLE: "대기",
  RUNNING: "경기 진행중",
  END: "경기 종료",
};

/**
 * 입력: ros(WS 연결 객체), color(로그인된 팀), connStatus(WS 연결 상태), onLogout(로그아웃 콜백).
 * 출력: 없음 — `/judge/match_state` 토픽을 구독해 경기상태/연결상태 배지를 렌더.
 */
export function MatchStateBanner({
  ros,
  color,
  connStatus,
  onLogout,
}: {
  ros: ROSLIB.Ros;
  color: TeamColor;
  connStatus: ConnectionStatus;
  onLogout: () => void;
}) {
  const matchState = useTopic<StringMsg>(ros, topics.matchState, "std_msgs/String");
  const state = matchState?.data ?? "IDLE";

  return (
    <header className={`banner banner--${color} state-${state}`}>
      <span className="banner__team">{color === "blue" ? "BLUE" : "RED"} 팀 PC</span>
      <span className="banner__state">{STATE_LABEL[state] ?? state}</span>
      <span className={`banner__conn banner__conn--${connStatus}`}>
        {connStatus === "connected" ? "● 연결됨" : connStatus === "connecting" ? "○ 연결중" : "○ 연결 끊김"}
      </span>
      <button className="banner__logout" onClick={onLogout}>
        로그아웃
      </button>
    </header>
  );
}
