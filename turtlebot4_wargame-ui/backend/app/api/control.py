"""심판 PC UI의 경기 시작/종료 버튼이 호출하는 컨트롤 API.

실제 배포 시에는 이 REST 트리거를 심판 PC의 match_manager ROS2 노드 호출로 바꾸면 된다.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from .. import config
from ..db import crud
from ..db.database import get_db
from ..mock.state import state
from ..rosbridge.hub import hub

router = APIRouter(prefix="/api/match", tags=["control"])


@router.post("/start")
async def start_match(db: Session = Depends(get_db)) -> dict:
    """경기 시작. 입력값 없음(body 불필요) — 현재 이미 RUNNING이면 409.
    처리: DB에 새 match row 생성 -> 인메모리 state 리셋(HP/battery 초기화) -> match_state=RUNNING 브로드캐스트.
    출력: {match_id, match_state}."""
    if state.match_state == "RUNNING":
        raise HTTPException(status_code=409, detail="match already running")

    match = crud.create_match(db)
    state.start(match.id)

    await hub.publish(config.MATCH_STATE_TOPIC, {"data": "RUNNING"})
    return {"match_id": match.id, "match_state": state.match_state}


@router.post("/end")
async def end_match(db: Session = Depends(get_db)) -> dict:
    """경기 종료. 입력값 없음 — RUNNING 상태가 아니면 409.
    처리: 생존 팀 기준 승자 계산 -> DB match row에 종료시각/승자 기록 -> match_state=END 브로드캐스트.
    출력: {match_id, match_state, winner}."""
    if state.match_state != "RUNNING" or state.match_id is None:
        raise HTTPException(status_code=409, detail="no match is running")

    winner = state.winner()
    crud.end_match(db, state.match_id, winner)
    state.end()

    await hub.publish(config.MATCH_STATE_TOPIC, {"data": "END"})
    return {"match_id": state.match_id, "match_state": state.match_state, "winner": winner}
