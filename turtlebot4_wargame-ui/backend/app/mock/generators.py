"""
더미데이터 생성기.

실제 로봇/센서/YOLO 대신 그럴듯한 값을 주기적으로 rosbridge 허브에 publish한다.
나중에 진짜 ROS2 노드가 붙으면 이 파일 전체를 걷어내면 된다(토픽 이름/스키마는 그대로 유지됨).
"""

from __future__ import annotations

import asyncio
import random
import time
from typing import Any

from .. import config
from ..db.database import get_session
from ..db import crud
from ..rosbridge.hub import hub
from .state import state

# 1x1 투명 PNG — 실제 카메라 영상 대신 쓰는 아주 가벼운 placeholder.
PLACEHOLDER_PNG_B64 = (
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)


def _rand_bbox() -> dict:
    x = round(random.uniform(0.05, 0.7), 3)
    y = round(random.uniform(0.05, 0.7), 3)
    return {
        "x": x,
        "y": y,
        "w": round(random.uniform(0.1, 0.25), 3),
        "h": round(random.uniform(0.1, 0.25), 3),
        "conf": round(random.uniform(0.55, 0.98), 2),
    }


async def team_status_loop(color: str) -> None:
    """1초마다 팀 하나의 상태를 갱신하고 `/{color}/status`로 publish하는 무한 루프.

    입력(필요한 값): `state.teams[color]`의 현재 self_hp/battery/robot_destroyed
    (self_hp·robot_destroyed는 실제로는 심판 판정/명중 이벤트가 갱신하고, 여기선 mission_state·battery만 스스로 굴린다).
    출력: {self_hp, battery, mission_state, robot_destroyed, timestamp} — 팀PC/심판PC UI의 StatusPanel이 그대로 구독.

    휴먼인더루프: WAIT_APPROVAL 상태는 팀 PC(AimPanel)에서 사람이 "발사 승인" 버튼을 눌러
    team.shoot_approved가 True가 되기 전까지는 FIRE로 넘어가지 않고 계속 대기한다.
    """
    cycle = ["CHASE", "AIM", "WAIT_APPROVAL", "RETURN"]
    tick = 0
    while True:
        await asyncio.sleep(1.0)
        tick += 1
        team = state.teams[color]

        if team.robot_destroyed:
            team.mission_state = "DISABLED"
        elif state.match_state != "RUNNING":
            team.mission_state = "IDLE"
        elif team.mission_state == "WAIT_APPROVAL":
            if team.shoot_approved:
                team.mission_state = "FIRE"
        elif team.mission_state == "FIRE":
            team.mission_state = "RETURN"
            team.shoot_approved = False
        else:
            if team.mission_state not in cycle:
                team.mission_state = cycle[0]  # IDLE 등에서 즉시 CHASE로 진입
            elif tick % 3 == 0:
                idx = cycle.index(team.mission_state)
                team.mission_state = cycle[(idx + 1) % len(cycle)]
            team.battery = max(0.0, team.battery - random.uniform(0.02, 0.08))

        await hub.publish(
            config.team_topic(color, "status"),
            {
                "self_hp": team.self_hp,
                "battery": round(team.battery, 1),
                "mission_state": team.mission_state,
                "robot_destroyed": team.robot_destroyed,
                "timestamp": time.time(),
            },
        )


async def aim_loop(color: str) -> None:
    """조준 관련 4개 토픽을 흉내내는 루프 — mission_state가 AIM/WAIT_APPROVAL/FIRE일 때만 동작.

    입력: `state.teams[color].mission_state` (team_status_loop가 갱신한 값을 읽기만 함).
    출력: aim/set_ori(각도), target/xyz(좌표), aim/complete(각도<3도면 true), shoot_approval(FIRE 상태면 true).
    실제 연동 시엔 이 4개를 vision/aim 담당 ROS2 노드가 발행하게 되고 이 함수는 통째로 삭제된다.
    """
    while True:
        await asyncio.sleep(1.5)
        team = state.teams[color]
        if team.mission_state not in ("AIM", "WAIT_APPROVAL", "FIRE"):
            continue

        set_ori = round(random.uniform(-30, 30), 2)
        await hub.publish(config.AIM_SET_ORI_TOPIC(color), {"data": set_ori})
        await hub.publish(
            config.TARGET_XYZ_TOPIC(color),
            {"x": round(random.uniform(0.3, 1.0), 2), "y": round(random.uniform(-0.3, 0.3), 2), "z": 0.0},
        )
        await hub.publish(config.AIM_COMPLETE_TOPIC(color), {"data": abs(set_ori) < 3})
        await hub.publish(config.SHOOT_APPROVAL_TOPIC(color), {"data": team.mission_state == "FIRE"})


async def camera_loop(color: str) -> None:
    while True:
        await asyncio.sleep(2.0)
        detections = [_rand_bbox() for _ in range(random.choice([0, 0, 1]))]
        for det in detections:
            det["label"] = "enemy_robot"
        await hub.publish(
            config.CAMERA_TOPIC(color),
            {"format": "png", "data": PLACEHOLDER_PNG_B64, "mock": True},
        )
        await hub.publish(config.ENEMY_DETECTIONS_TOPIC(color), {"detections": detections})


async def judge_camera_loop() -> None:
    while True:
        await asyncio.sleep(2.0)
        detections = []
        if random.random() < 0.8:
            detections.append({**_rand_bbox(), "label": "robot_blue"})
        if random.random() < 0.8:
            detections.append({**_rand_bbox(), "label": "robot_red"})
        if random.random() < 0.3:
            detections.append({**_rand_bbox(), "label": "ball"})

        await hub.publish(config.JUDGE_IMAGE_TOPIC, {"format": "png", "data": PLACEHOLDER_PNG_B64, "mock": True})
        await hub.publish(config.JUDGE_DETECTIONS_TOPIC, {"detections": detections})


async def match_state_loop() -> None:
    while True:
        await asyncio.sleep(1.0)
        await hub.publish(config.MATCH_STATE_TOPIC, {"data": state.match_state})


async def hit_event_loop() -> None:
    """5~15초 랜덤 간격으로 생존한 팀 중 하나를 골라 명중 처리하는 루프 — 실제 명중 판정(YOLO+심판) 대체 목업.

    입력: `state.match_state`(RUNNING이어야 동작), `state.match_id`(DB에 기록할 경기 id), 생존 팀 목록.
    출력: 1) `state.apply_hit()`으로 HP 차감(인메모리), 2) DB에 hit_event row 저장(blue/red HP 스냅샷 포함),
    3) `/judge/hit_event` publish, 4) HP 0이 되면 `/{color}/robot_destroyed` publish.
    이 함수가 "필요한 값 -> DB 기록 -> 실시간 publish"까지 이어지는 전체 파이프라인의 핵심 예시.
    """
    next_hit_at = time.time() + random.uniform(5, 15)
    while True:
        await asyncio.sleep(1.0)
        if state.match_state != "RUNNING" or state.match_id is None:
            continue
        if time.time() < next_hit_at:
            continue

        alive_colors = [c for c in config.TEAM_COLORS if not state.teams[c].robot_destroyed]
        if not alive_colors:
            next_hit_at = time.time() + 10
            continue

        victim_color = random.choice(alive_colors)
        state.apply_hit(victim_color)
        next_hit_at = time.time() + random.uniform(5, 15)

        blue_hp = state.teams["blue"].self_hp
        red_hp = state.teams["red"].self_hp

        db = get_session()
        try:
            crud.add_hit_event(db, state.match_id, victim_color, blue_hp, red_hp)
        finally:
            db.close()

        await hub.publish(config.HIT_EVENT_TOPIC, {"victim": victim_color, "timestamp": time.time()})
        if state.teams[victim_color].robot_destroyed:
            await hub.publish(config.ROBOT_DESTROYED_TOPIC(victim_color), {"data": True})


def _make_shoot_approval_handler(color: str):
    """팀 PC가 `/{color}/shoot_approval`에 {"data": true}를 publish(사람이 버튼 클릭)하면
    인메모리 state에 반영 — 휴먼인더루프의 핵심 연결고리. hub.py의 on_publish 훅으로 등록된다."""

    def handler(_topic: str, msg: Any) -> None:
        if isinstance(msg, dict) and msg.get("data") is True:
            state.teams[color].shoot_approved = True

    return handler


def register_publish_handlers() -> None:
    """클라이언트(팀 PC)가 publish하는 토픽을 서버 상태에 반영하기 위한 훅 등록.
    main.py의 lifespan()이 서버 시작 시 한 번 호출."""
    for color in config.TEAM_COLORS:
        hub.on_publish(config.SHOOT_APPROVAL_TOPIC(color), _make_shoot_approval_handler(color))


def start_background_tasks() -> list[asyncio.Task]:
    """위 루프들을 asyncio task로 띄워서 리스트로 반환 — main.py의 lifespan()이 서버 시작 시 호출.
    judge_camera_loop는 judge-vision/judge_vision.py(실제 웹캠+YOLO)가 같은 토픽을 publish하므로 비활성화."""
    register_publish_handlers()
    tasks: list[asyncio.Task] = [asyncio.create_task(match_state_loop()), asyncio.create_task(hit_event_loop())]
    for color in config.TEAM_COLORS:
        tasks.append(asyncio.create_task(team_status_loop(color)))
        tasks.append(asyncio.create_task(aim_loop(color)))
        tasks.append(asyncio.create_task(camera_loop(color)))
    return tasks


async def stop_background_tasks(tasks: list[asyncio.Task]) -> None:
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
