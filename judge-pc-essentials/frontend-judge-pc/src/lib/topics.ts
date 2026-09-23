/**
 * 토픽 이름 상수 — backend/app/config.py 와 반드시 대응되어야 함.
 * 백엔드에서 토픽 이름을 바꾸면 여기도 같이 바꿀 것.
 */
export type TeamColor = "blue" | "red";

export const teamTopic = (color: TeamColor, suffix: string) => `/${color}/${suffix}`;

export const topics = {
  status: (c: TeamColor) => teamTopic(c, "status"), // self_hp/battery/mission_state/robot_destroyed
  robotDestroyed: (c: TeamColor) => teamTopic(c, "robot_destroyed"),
  matchState: "/judge/match_state",
  hitEvent: "/judge/hit_event",
  judgeImage: "/judge/image_raw",
  judgeDetections: "/judge/detections",
} as const;

export interface TeamStatusMsg {
  self_hp: number;
  battery: number;
  mission_state: string;
  robot_destroyed: boolean;
  timestamp: number;
}

export interface BoolMsg {
  data: boolean;
}

export interface StringMsg {
  data: string;
}

export interface Detection {
  x: number;
  y: number;
  w: number;
  h: number;
  conf: number;
  label: string;
}

export interface DetectionsMsg {
  detections: Detection[];
}

export interface CompressedImageMsg {
  format: string;
  data: string; // base64
  mock?: boolean;
}

export interface HitEventMsg {
  victim: TeamColor;
  timestamp: number;
}
