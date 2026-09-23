"""가짜 rosbridge WebSocket 엔드포인트.

roslibjs가 `new ROSLIB.Ros({url: "ws://.../ws"})`로 접속하면 이 핸들러가 받는다.
실제 rosbridge_server로 교체할 때는 프론트의 접속 URL만 바꾸면 되고,
클라이언트 코드는 수정할 필요가 없도록 프로토콜 형태를 최대한 맞춘다.
"""

from __future__ import annotations

import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .hub import hub
from .protocol import ProtocolError, parse, status_message

logger = logging.getLogger("rosbridge.server")

router = APIRouter()


@router.websocket("/ws")
async def rosbridge_ws(websocket: WebSocket) -> None:
    """입력: 클라이언트가 보내는 rosbridge JSON({op, topic, msg, ...}) 연속 스트림.
    출력: 없음(양방향 WS) — op에 따라 hub.subscribe/unsubscribe/publish로 위임하고,
    에러/미지원 op는 status 메시지로 되돌려줌."""
    await websocket.accept()
    try:
        while True:
            raw = await websocket.receive_json()
            try:
                message = parse(raw)
            except ProtocolError as exc:
                await websocket.send_json(status_message("error", str(exc)))
                continue

            if message.op == "subscribe" and message.topic:
                await hub.subscribe(message.topic, websocket)
            elif message.op == "unsubscribe" and message.topic:
                await hub.unsubscribe(message.topic, websocket)
            elif message.op in ("advertise", "unadvertise"):
                pass  # 이 목업에선 별도 등록이 필요 없음
            elif message.op == "publish" and message.topic:
                await hub.publish(message.topic, message.msg)
            elif message.op == "call_service":
                await websocket.send_json(
                    status_message("error", f"service '{message.service}' not implemented in mock server", message.id)
                )
            else:
                await websocket.send_json(status_message("warning", f"unhandled op '{message.op}'"))
    except WebSocketDisconnect:
        pass
    finally:
        await hub.unsubscribe_all(websocket)
