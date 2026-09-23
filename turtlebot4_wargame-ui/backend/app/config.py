"""
토픽/메시지 계약의 단일 소스.

시스템 흐름도(drawio)에 나온 토픽은 그대로,
프론트 렌더링에 필요한 토픽 일부는 새로 정의함.
프론트(frontend-*/src/lib/topics.ts)는 이 파일의 이름과 반드시 맞춰야 한다.
"""

from __future__ import annotations

TEAM_COLORS = ("blue", "red")


def team_topic(color: str, suffix: str) -> str:
    assert color in TEAM_COLORS
    return f"/{color}/{suffix}"


# ---- 팀 PC 관련 토픽 (drawio) ----
TEAM_STATUS_TOPIC = team_topic  # 사용: team_topic(color, "status") -> "/blue/status"
AIM_SET_ORI_TOPIC = lambda c: team_topic(c, "aim/set_ori")  # noqa: E731  Float32
TARGET_XYZ_TOPIC = lambda c: team_topic(c, "target/xyz")  # noqa: E731  PointStamped
AIM_COMPLETE_TOPIC = lambda c: team_topic(c, "aim/complete")  # noqa: E731  Bool
SHOOT_CMD_TOPIC = lambda c: team_topic(c, "shoot_cmd")  # noqa: E731  String
SHOOT_APPROVAL_TOPIC = lambda c: team_topic(c, "shoot_approval")  # noqa: E731  Bool
ROBOT_DESTROYED_TOPIC = lambda c: team_topic(c, "robot_destroyed")  # noqa: E731  Bool
CMD_VEL_TOPIC = lambda c: team_topic(c, "cmd_vel")  # noqa: E731  Twist
CAMERA_TOPIC = lambda c: team_topic(c, "enemy_image/compressed")  # noqa: E731  CompressedImage(실제 YOLO 노드/topics.ts와 일치)
ENEMY_DETECTIONS_TOPIC = lambda c: team_topic(c, "enemy/detections")  # noqa: E731 bbox 목록(JSON)

# ---- 심판 PC 관련 토픽 (drawio) ----
MATCH_STATE_TOPIC = "/judge/match_state"  # String: "IDLE" | "RUNNING" | "END"
HIT_EVENT_TOPIC = "/judge/hit_event"  # {victim: "blue"|"red", timestamp}
JUDGE_IMAGE_TOPIC = "/judge/image_raw"  # placeholder image
JUDGE_DETECTIONS_TOPIC = "/judge/detections"  # robot_blue/robot_red/ball bbox 목록(JSON)

# 원래 diagram엔 self_hp/mission_state/battery가 "j_tstat"로 뭉뚱그려져 있어
# 개별 토픽 대신 팀당 하나의 status 토픽으로 통합함(비전/펌웨어 팀과 최종 스키마 협의 필요).
MISSION_STATES = ("IDLE", "CHASE", "AIM", "WAIT_APPROVAL", "FIRE", "RETURN", "DISABLED")

STARTING_HP = 3
