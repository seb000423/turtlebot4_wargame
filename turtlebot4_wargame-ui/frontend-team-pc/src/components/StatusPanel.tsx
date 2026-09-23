import type ROSLIB from "roslib";
import { useTopic } from "../lib/ros";
import { MAX_AMMO, topics, type TeamColor, type TeamStatusMsg } from "../lib/topics";

/**
 * 입력: ros, color — `/{color}/status` 토픽 하나만 구독.
 * 출력: 없음 — self_hp(HP 3칸), ammo(장전 N/5 + 탄 표시), reload_remaining(재장전 카운트다운),
 * mission_state(뱃지), robot_destroyed(경고문)를 렌더.
 */
export function StatusPanel({ ros, color }: { ros: ROSLIB.Ros; color: TeamColor }) {
  const status = useTopic<TeamStatusMsg>(ros, topics.status(color), "turtlebot4_wargame_msgs/TeamStatus");
  const reloadSec = status ? Math.ceil(status.reload_remaining) : 0;

  return (
    <section className="panel">
      <h2>로봇 상태</h2>
      {!status ? (
        <p className="muted">데이터 수신 대기중…</p>
      ) : (
        <div className="status-grid">
          <div className="stat">
            <span className="stat__label">HP</span>
            <div className="hp-bar">
              {Array.from({ length: 3 }).map((_, i) => (
                <span key={i} className={`hp-cell ${i < status.self_hp ? "hp-cell--full" : ""}`} />
              ))}
            </div>
          </div>
          <div className="stat">
            <span className="stat__label">장전</span>
            <span className="ammo">
              <span className="ammo-bar">
                {Array.from({ length: MAX_AMMO }).map((_, i) => (
                  <span key={i} className={`ammo-cell ${i < status.ammo ? "ammo-cell--live" : ""}`} />
                ))}
              </span>
              <span className="stat__value">
                {status.ammo} / {MAX_AMMO}
              </span>
            </span>
          </div>
          {reloadSec > 0 && (
            <div className="stat">
              <span className="stat__label">재장전</span>
              <span className="badge badge--wait reload-badge">재장전 중 {reloadSec}초</span>
            </div>
          )}
          <div className="stat">
            <span className="stat__label">임무 상태</span>
            <span className={`badge badge--${status.mission_state.toLowerCase()}`}>{status.mission_state}</span>
          </div>
          {status.robot_destroyed && <div className="alert">SYSTEM DISABLED</div>}
        </div>
      )}
    </section>
  );
}
