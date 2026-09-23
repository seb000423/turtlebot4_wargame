"""
토픽 이름 -> 구독중인 WebSocket 목록을 관리하고 브로드캐스트하는 허브.

실제 rosbridge_server의 역할(구독자 관리 + publish 라우팅)만 흉내낸다.
"""

from __future__ import annotations

import asyncio
import logging
from collections import defaultdict
from typing import Any, Awaitable, Callable

from fastapi import WebSocket

logger = logging.getLogger("rosbridge.hub")

PublishHandler = Callable[[str, Any], Awaitable[None] | None]


class RosbridgeHub:
    def __init__(self) -> None:
        self._subscribers: dict[str, set[WebSocket]] = defaultdict(set)
        self._lock = asyncio.Lock()
        # 클라이언트가 특정 토픽에 publish 했을 때 목업 상태를 갱신하고 싶은 경우를 위한 훅.
        self._publish_handlers: dict[str, list[PublishHandler]] = defaultdict(list)

    def on_publish(self, topic: str, handler: PublishHandler) -> None:
        self._publish_handlers[topic].append(handler)

    async def subscribe(self, topic: str, ws: WebSocket) -> None:
        async with self._lock:
            self._subscribers[topic].add(ws)
        logger.info("subscribe %s (subscribers=%d)", topic, len(self._subscribers[topic]))

    async def unsubscribe(self, topic: str, ws: WebSocket) -> None:
        async with self._lock:
            self._subscribers[topic].discard(ws)

    async def unsubscribe_all(self, ws: WebSocket) -> None:
        async with self._lock:
            for subs in self._subscribers.values():
                subs.discard(ws)

    async def publish(self, topic: str, msg: Any) -> None:
        """서버 쪽 목업 생성기 및 클라이언트 publish 모두 이 경로로 브로드캐스트한다.

        입력: topic(문자열), msg(그 토픽 스키마에 맞는 dict — 예: {"data": "RUNNING"}).
        출력: 없음(리턴값 X) — 대신 해당 topic을 구독중인 모든 WebSocket에 {op:"publish", topic, msg} JSON을 전송."""
        for handler in self._publish_handlers.get(topic, []):
            result = handler(topic, msg)
            if asyncio.iscoroutine(result):
                await result

        async with self._lock:
            targets = list(self._subscribers.get(topic, ()))

        if not targets:
            return

        payload = {"op": "publish", "topic": topic, "msg": msg}
        dead: list[WebSocket] = []
        for ws in targets:
            try:
                await ws.send_json(payload)
            except Exception:  # noqa: BLE001 - 끊어진 소켓은 정리만 하면 됨
                dead.append(ws)

        if dead:
            async with self._lock:
                for ws in dead:
                    self._subscribers[topic].discard(ws)


hub = RosbridgeHub()
