"""
rosbridge v2 JSON 프로토콜 중 이 프로젝트에서 필요한 부분만 최소 구현.
(참고: https://github.com/RobotWebTools/rosbridge_suite protocol spec)

roslibjs 클라이언트가 보내는 op:
- subscribe / unsubscribe   : {op, topic, id?, type?}
- advertise / unadvertise   : {op, topic, type?}
- publish                   : {op, topic, msg}
- call_service              : {op, service, id?, args?}  (이 프로젝트에서는 미사용, 에러로 응답)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


class ProtocolError(ValueError):
    pass


@dataclass
class RosMessage:
    op: str
    topic: str | None = None
    msg: Any = None
    id: str | None = None
    service: str | None = None
    args: Any = None


def parse(raw: dict) -> RosMessage:
    """입력: 클라이언트가 보낸 raw JSON dict. 출력: RosMessage(op 필수, 나머지는 optional). op 없으면 예외."""
    if "op" not in raw:
        raise ProtocolError("missing 'op' field")
    return RosMessage(
        op=raw["op"],
        topic=raw.get("topic"),
        msg=raw.get("msg"),
        id=raw.get("id"),
        service=raw.get("service"),
        args=raw.get("args"),
    )


def status_message(level: str, msg: str, id_: str | None = None) -> dict:
    """rosbridge의 status op — 에러/경고를 클라이언트에 알릴 때 사용."""
    out: dict[str, Any] = {"op": "status", "level": level, "msg": msg}
    if id_ is not None:
        out["id"] = id_
    return out
