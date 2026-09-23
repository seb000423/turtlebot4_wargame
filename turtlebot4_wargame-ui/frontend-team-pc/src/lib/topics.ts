/**
 * 토픽 이름 상수 — backend/app/config.py 와 반드시 대응되어야 함.
 * 백엔드에서 토픽 이름을 바꾸면 여기도 같이 바꿀 것.
 */
export type TeamColor = "blue" | "red";

// UI 라벨/로그인("BLUE"/"RED", blue1234/red1234)은 그대로 두고, 실제 ROS
// 토픽 prefix만 이 매핑을 거친다. 지금 물리 로봇이 아직 /robot4 네임스페이스로
// 떠 있어서 blue 팀을 그쪽으로 연결해둔 것 — 나중에 로봇을 정식으로
// /blue 로 재구성하면 { blue: "blue", red: "red" } 로 되돌리면 된다.
const ROS_NAMESPACE: Record<TeamColor, string> = {
  blue: "robot4",
  red: "red",
};

export const teamTopic = (color: TeamColor, suffix: string) => `/${ROS_NAMESPACE[color]}/${suffix}`;

export const topics = {
  status: (c: TeamColor) => teamTopic(c, "status"), // self_hp/battery/mission_state/robot_destroyed
  aimSetOri: (c: TeamColor) => teamTopic(c, "aim/set_ori"),
  targetXyz: (c: TeamColor) => teamTopic(c, "target/xyz"),
  aimComplete: (c: TeamColor) => teamTopic(c, "aim/complete"),
  shootApproval: (c: TeamColor) => teamTopic(c, "shoot_approval"),
  robotDestroyed: (c: TeamColor) => teamTopic(c, "robot_destroyed"),
  // 탐지 노드(OAK-D 기반 bbox4.py)가 발행하는 압축 이미지 토픽.
  camera: (c: TeamColor) => teamTopic(c, "enemy_image/compressed"),
  enemyDetections: (c: TeamColor) => teamTopic(c, "enemy/detections"),
  matchState: "/judge/match_state",
  hitEvent: "/judge/hit_event",
} as const;

// 총 장전 탄수 — mission_manager.py의 MAX_AMMO 와 맞춰야 함.
export const MAX_AMMO = 5;

export interface TeamStatusMsg {
  self_hp: number;
  battery: number;
  ammo: number; // 남은 장전 탄수(0~MAX_AMMO)
  reload_remaining: number; // 재장전 남은 초(0이면 발사 가능)
  mission_state: string;
  robot_destroyed: boolean;
  timestamp: number;
}

export interface Float32Msg {
  data: number;
}

export interface BoolMsg {
  data: boolean;
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

export interface StringMsg {
  data: string;
}
