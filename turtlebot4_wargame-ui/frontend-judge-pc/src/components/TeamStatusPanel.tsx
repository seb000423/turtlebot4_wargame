import type ROSLIB from "roslib";
import { useTopic } from "../lib/ros";
import { topics, type TeamColor, type TeamStatusMsg } from "../lib/topics";

/**
 * 입력: ros, color — `/{color}/status` 토픽 구독(team-pc의 StatusPanel과 동일 토픽, 심판 관제용 축약 표시).
 * 출력: 없음 — HP(x/3), 배터리(%), mission_state, robot_destroyed 시 DISABLED 경고를 렌더.
 */
export function TeamStatusPanel({ ros, color }: { ros: ROSLIB.Ros; color: TeamColor }) {
  const status = useTopic<TeamStatusMsg>(ros, topics.status(color), "turtlebot4_wargame_msgs/TeamStatus");

  return (
    <section className={`panel team-status team-status--${color}`}>
      <h2>{color === "blue" ? "BLUE" : "RED"} 팀 PC</h2>
      {!status ? (
        <p className="muted">데이터 수신 대기중…</p>
      ) : (
        <div className="status-grid">
          <div className="stat">
            <span className="stat__label">HP</span>
            <span className="stat__value">{status.self_hp} / 3</span>
          </div>
          <div className="stat">
            <span className="stat__label">배터리</span>
            <span className="stat__value">{status.battery.toFixed(0)}%</span>
          </div>
          <div className="stat">
            <span className="stat__label">임무 상태</span>
            <span className="badge">{status.mission_state}</span>
          </div>
          {status.robot_destroyed && <div className="alert">DISABLED</div>}
        </div>
      )}
    </section>
  );
}
