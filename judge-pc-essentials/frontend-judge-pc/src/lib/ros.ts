import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

const ROSBRIDGE_URL = import.meta.env.VITE_ROSBRIDGE_URL ?? "ws://localhost:8000/ws";

export type ConnectionStatus = "connecting" | "connected" | "error" | "closed";

/**
 * 가짜 rosbridge든 진짜 rosbridge_server든 동일하게 접속한다.
 * 실제 장비 연동 시엔 .env의 VITE_ROSBRIDGE_URL만 바꾸면 이 파일은 그대로 재사용 가능.
 * 입력: 없음(props X) — .env의 VITE_ROSBRIDGE_URL(기본 ws://localhost:8000/ws).
 * 출력: {ros, status} — ros는 하위 컴포넌트들이 useTopic/useTopicLog에 넘기는 연결 객체.
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
 * 입력: ros, topicName, messageType. 출력: 최신 메시지(T) 또는 아직 없으면 undefined.
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
 * 여러 개가 쌓여야 하는(로그성) 토픽용 — 최신 N개를 배열로 유지.
 * 입력: ros, topicName, messageType, maxLen(최대 보관 개수). 출력: 최신순 배열(길이 <= maxLen).
 */
export function useTopicLog<T>(ros: ROSLIB.Ros, topicName: string, messageType: string, maxLen = 20): T[] {
  const [log, setLog] = useState<T[]>([]);

  useEffect(() => {
    const topic = new ROSLIB.Topic({ ros, name: topicName, messageType });
    const handler = (msg: unknown) => setLog((prev) => [msg as T, ...prev].slice(0, maxLen));
    topic.subscribe(handler);
    return () => topic.unsubscribe(handler);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ros, topicName]);

  return log;
}
