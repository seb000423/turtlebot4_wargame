import type ROSLIB from "roslib";
import { useTopic } from "../lib/ros";
import { topics, type CompressedImageMsg, type DetectionsMsg } from "../lib/topics";

const LABEL_CLASS: Record<string, string> = {
  robot_blue: "bbox--blue",
  robot_red: "bbox--red",
  ball: "bbox--ball",
};

/**
 * 입력: ros — `/judge/image_raw`(이미지), `/judge/detections`(robot_blue/robot_red/ball bbox 배열) 구독.
 * 출력: 없음 — 이미지 + label별 색상(blue/red/ball) 다른 bbox 오버레이를 렌더.
 */
export function JudgeCameraFeed({ ros }: { ros: ROSLIB.Ros }) {
  const image = useTopic<CompressedImageMsg>(ros, topics.judgeImage, "sensor_msgs/CompressedImage");
  const detections = useTopic<DetectionsMsg>(ros, topics.judgeDetections, "turtlebot4_wargame_msgs/Detections");

  return (
    <section className="panel">
      <h2>심판 카메라 (YOLO 판정)</h2>
      <div className="camera-view">
        {image ? (
          <img
            className="camera-view__img"
            src={`data:image/${image.format};base64,${image.data}`}
            alt="judge camera feed"
          />
        ) : (
          <div className="camera-view__placeholder">신호 없음</div>
        )}
        {image?.mock && <span className="mock-tag">MOCK FEED</span>}
        {detections?.detections.map((d, i) => (
          <div
            key={i}
            className={`bbox ${LABEL_CLASS[d.label] ?? ""}`}
            style={{
              left: `${d.x * 100}%`,
              top: `${d.y * 100}%`,
              width: `${d.w * 100}%`,
              height: `${d.h * 100}%`,
            }}
          >
            <span className="bbox__label">
              {d.label} {(d.conf * 100).toFixed(0)}%
            </span>
          </div>
        ))}
      </div>
    </section>
  );
}
