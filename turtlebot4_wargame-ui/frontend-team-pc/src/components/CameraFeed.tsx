import type ROSLIB from "roslib";
import { useTopic } from "../lib/ros";
import { topics, type CompressedImageMsg, type DetectionsMsg, type TeamColor } from "../lib/topics";

/**
 * 입력: ros, color — `/{color}/enemy_image/compressed`(이미지, OAK-D), `/{color}/enemy/detections`(bbox 배열) 구독.
 * 출력: 없음 — base64 이미지를 <img>로, detections를 좌표(x,y,w,h 0~1 비율) 기반 오버레이 박스로 렌더.
 */
export function CameraFeed({ ros, color }: { ros: ROSLIB.Ros; color: TeamColor }) {
  const image = useTopic<CompressedImageMsg>(ros, topics.camera(color), "sensor_msgs/CompressedImage");
  const detections = useTopic<DetectionsMsg>(ros, topics.enemyDetections(color), "turtlebot4_wargame_msgs/Detections");

  return (
    <section className="panel">
      <h2>카메라 (OAK-D)</h2>
      <div className="camera-view">
        {image ? (
          <img
            className="camera-view__img"
            src={`data:image/${image.format};base64,${image.data}`}
            alt="camera feed"
          />
        ) : (
          <div className="camera-view__placeholder">신호 없음</div>
        )}
        {image?.mock && <span className="mock-tag">MOCK FEED</span>}
        {detections?.detections.map((d, i) => (
          <div
            key={i}
            className="bbox"
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
