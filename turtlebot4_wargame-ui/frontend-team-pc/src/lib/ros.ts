import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

const ROSBRIDGE_URL = import.meta.env.VITE_ROSBRIDGE_URL ?? "ws://localhost:8000/ws";

export type ConnectionStatus = "connecting" | "connected" | "error" | "closed";

/**
 * 가짜 rosbridge든 진짜 rosbridge_server든 동일하게 접속한다.
 * 실제 장비 연동 시엔 .env의 VITE_ROSBRIDGE_URL만 바꾸면 이 파일은 그대로 재사용 가능.
 * 입력: 없음(props X) — .env의 VITE_ROSBRIDGE_URL(기본 ws://localhost:8000/ws).
 * 출력: {ros, status} — ros는 하위 컴포넌트들이 useTopic에 넘기는 연결 객체, status는 배너의 연결 표시에 사용.
 */
export function useRos(): { ros: ROSLIB.Ros; status: ConnectionStatus } {
  const rosRef = useRef<ROSLIB.Ros>();
  const [status, setStatus] = useState<ConnectionStatus>("connecting");

  if (!rosRef.current) {
    rosRef.current = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
  }

  useEffect(() => {
    const ros = rosRef.current!;
    const onConnect = () => setStatus("connected");
    const onError = () => setStatus("error");
    const onClose = () => setStatus("closed");

    ros.on("connection", onConnect);
    ros.on("error", onError);
    ros.on("close", onClose);

    return () => {
      ros.off("connection", onConnect);
      ros.off("error", onError);
      ros.off("close", onClose);
    };
  }, []);

  return { ros: rosRef.current, status };
}

/**
 * 토픽 하나를 구독해서 최신 메시지를 반환하는 훅.
 * 입력: ros, topicName(구독할 토픽), messageType(ROS 메시지 타입 문자열).
 * 출력: 해당 토픽의 가장 최근 메시지(T) — 아직 한 번도 안 왔으면 undefined.
 */
export function useTopic<T>(ros: ROSLIB.Ros, topicName: string, messageType = "std_msgs/String"): T | undefined {
  const [message, setMessage] = useState<T>();

  useEffect(() => {
    const topic = new ROSLIB.Topic({ ros, name: topicName, messageType });
    const handler = (msg: unknown) => setMessage(msg as T);
    topic.subscribe(handler);
    return () => topic.unsubscribe(handler);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ros, topicName]);

  return message;
}

/**
 * 토픽에 메시지 하나를 1회 publish한다 (사람의 버튼 클릭 등 UI → 서버 방향 액션용).
 * 입력: ros, topicName(발행할 토픽), msg(그 토픽 스키마에 맞는 값), messageType.
 * 출력: 없음 — 가짜 rosbridge든 진짜 rosbridge_server든 동일하게 동작(hub.py가 브로드캐스트/훅 처리).
 */
export function publishTopic(ros: ROSLIB.Ros, topicName: string, msg: unknown, messageType = "std_msgs/String"): void {
  const topic = new ROSLIB.Topic({ ros, name: topicName, messageType });
  topic.publish(msg as ROSLIB.Message);
}
