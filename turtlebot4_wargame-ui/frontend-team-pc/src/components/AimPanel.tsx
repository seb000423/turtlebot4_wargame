import { useEffect, useState } from "react";
import type ROSLIB from "roslib";
import { publishTopic, useTopic } from "../lib/ros";
import {
  topics,
  type BoolMsg,
  type DetectionsMsg,
  type Float32Msg,
  type TeamColor,
  type TeamStatusMsg,
} from "../lib/topics";

/**
 * 입력: ros, color — aim/set_ori, aim/complete, shoot_approval, status, enemy/detections 토픽 구독.
 * 출력: 없음 — 조준각/정렬/승인 상태를 뱃지로 렌더 + 적 로봇이 감지되고 승인 대기(WAIT_APPROVAL) 중이면
 * "발사 승인" 버튼을 노출한다(휴먼인더루프). 버튼 클릭 시 `/{color}/shoot_approval`에 {data:true}를 publish.
 */
export function AimPanel({ ros, color }: { ros: ROSLIB.Ros; color: TeamColor }) {
  const setOri = useTopic<Float32Msg>(ros, topics.aimSetOri(color), "std_msgs/Float32");
  const aimComplete = useTopic<BoolMsg>(ros, topics.aimComplete(color), "std_msgs/Bool");
  const shootApproval = useTopic<BoolMsg>(ros, topics.shootApproval(color), "std_msgs/Bool");
  const status = useTopic<TeamStatusMsg>(ros, topics.status(color), "turtlebot4_wargame_msgs/TeamStatus");
  const detections = useTopic<DetectionsMsg>(ros, topics.enemyDetections(color), "turtlebot4_wargame_msgs/Detections");
  const [sending, setSending] = useState(false);

  const enemyDetected = (detections?.detections.length ?? 0) > 0;
  const awaitingApproval = status?.mission_state === "WAIT_APPROVAL" && !shootApproval?.data;
  const reloading = (status?.reload_remaining ?? 0) > 0;
  const showApproveButton = enemyDetected && awaitingApproval;

  // 승인 대기 상태를 벗어나면(승인 처리됨/임무 상태 변경됨) 전송중 표시를 초기화
  useEffect(() => {
    if (!awaitingApproval) setSending(false);
  }, [awaitingApproval]);

  const approveShot = () => {
    setSending(true);
    publishTopic(ros, topics.shootApproval(color), { data: true }, "std_msgs/Bool");
  };

  return (
    <>
      {/* 발사 승인 대기 중이면 화면 전체에 응급 상황처럼 빨간 경광등이 번쩍인다. */}
      {showApproveButton && <div className="emergency-flash" aria-hidden="true" />}
      <section className="panel">
        <h2>조준 / 발사</h2>
        <div className="status-grid">
          <div className="stat">
            <span className="stat__label">set_ori</span>
            <span className="stat__value">{setOri ? `${setOri.data.toFixed(1)}°` : "—"}</span>
          </div>
          <div className="stat">
            <span className="stat__label">조준 정렬</span>
            <span className={`badge ${aimComplete?.data ? "badge--ok" : "badge--wait"}`}>
              {aimComplete?.data ? "정렬됨" : "정렬중"}
            </span>
          </div>
          <div className="stat">
            <span className="stat__label">발사 승인</span>
            <span className={`badge ${shootApproval?.data ? "badge--ok" : "badge--wait"}`}>
              {shootApproval?.data ? "승인" : "대기"}
            </span>
          </div>
        </div>
        {showApproveButton && (
          <div className="approve-box">
            <p className="approve-box__msg">
              {reloading ? "재장전 중 — 잠시 후 발사할 수 있습니다" : "적 로봇 감지됨 — 발사 승인이 필요합니다"}
            </p>
            <button className="approve-box__btn" onClick={approveShot} disabled={sending || reloading}>
              {reloading ? `재장전 중 ${Math.ceil(status!.reload_remaining)}초` : sending ? "승인 전송 중…" : "발사 승인"}
            </button>
          </div>
        )}
      </section>
    </>
  );
}
