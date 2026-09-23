import type ROSLIB from "roslib";
import { useTopic } from "../lib/ros";
import { topics, type StringMsg, type TeamStatusMsg } from "../lib/topics";

const STATE_LABEL: Record<string, string> = {
  IDLE: "대기",
  RUNNING: "경기 진행중",
  END: "경기 종료",
};

function HpDots({ hp }: { hp: number }) {
  return (
    <div className="hp-bar">
      {Array.from({ length: 3 }).map((_, i) => (
        <span key={i} className={`hp-cell ${i < hp ? "hp-cell--full" : ""}`} />
      ))}
    </div>
  );
}

/**
 * 입력: ros — `/judge/match_state`, `/blue/status`, `/red/status` 3개 토픽 구독.
 * 출력: 없음 — 경기상태 + 양팀 HP(3칸 도트)/mission_state를 나란히 렌더.
 */
export function ScoreBoard({ ros }: { ros: ROSLIB.Ros }) {
  const matchState = useTopic<StringMsg>(ros, topics.matchState, "std_msgs/String");
  const blue = useTopic<TeamStatusMsg>(ros, topics.status("blue"), "turtlebot4_wargame_msgs/TeamStatus");
  const red = useTopic<TeamStatusMsg>(ros, topics.status("red"), "turtlebot4_wargame_msgs/TeamStatus");
  const state = matchState?.data ?? "IDLE";

  return (
    <section className="panel scoreboard">
      <div className="scoreboard__state">{STATE_LABEL[state] ?? state}</div>
      <div className="scoreboard__row">
        <div className="scoreboard__team scoreboard__team--blue">
          <span className="scoreboard__label">BLUE</span>
          <HpDots hp={blue?.self_hp ?? 3} />
          <span className="badge">{blue?.mission_state ?? "—"}</span>
        </div>
        <div className="scoreboard__vs">VS</div>
        <div className="scoreboard__team scoreboard__team--red">
          <span className="scoreboard__label">RED</span>
          <HpDots hp={red?.self_hp ?? 3} />
          <span className="badge">{red?.mission_state ?? "—"}</span>
        </div>
      </div>
    </section>
  );
}
