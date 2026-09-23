import type ROSLIB from "roslib";
import { useTopicLog } from "../lib/ros";
import { topics, type HitEventMsg } from "../lib/topics";

/**
 * 입력: ros — `/judge/hit_event` 토픽을 useTopicLog로 구독(최근 15개 누적).
 * 출력: 없음 — {victim, timestamp} 배열을 최신순 리스트로 렌더.
 */
export function HitEventLog({ ros }: { ros: ROSLIB.Ros }) {
  const hits = useTopicLog<HitEventMsg>(ros, topics.hitEvent, "turtlebot4_wargame_msgs/HitEvent", 15);

  return (
    <section className="panel">
      <h2>명중 이벤트</h2>
      {hits.length === 0 ? (
        <p className="muted">아직 명중 이벤트 없음</p>
      ) : (
        <ul className="hit-log">
          {hits.map((h, i) => (
            <li key={i} className={`hit-log__item hit-log__item--${h.victim}`}>
              <span className="hit-log__time">{new Date(h.timestamp * 1000).toLocaleTimeString()}</span>
              <span className="hit-log__text">{h.victim.toUpperCase()} 피격</span>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
