"""
토픽 이름 상수 — frontend-judge-pc/src/lib/topics.ts, frontend-team-pc/src/lib/topics.ts 와
반드시 대응되어야 함. 여기서 바꾸면 두 프론트엔드의 topics.ts도 같이 바꿀 것.
"""

MATCH_STATE_TOPIC = "/judge/match_state"
HIT_EVENT_TOPIC = "/judge/hit_event"


def team_status_topic(color: str) -> str:
    return f"/{color}/status"


# STATE_LABEL 키와 정확히 맞아야 함 (frontend-judge-pc/team-pc의 STATE_LABEL 참고)
STATE_IDLE = "IDLE"
STATE_RUNNING = "RUNNING"
STATE_END = "END"

MAX_HP = 3
TEAM_COLORS = ("red", "blue")
