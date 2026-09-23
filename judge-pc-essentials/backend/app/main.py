"""
심판 PC 백엔드: frontend-judge-pc가 호출하는 REST(api.ts)를 구현하고, "경기 시작/종료" 버튼
입력을 실제 ROS2 토픽(/judge/match_state) publish로 내보낸다.

이게 곧 "레드 PC / 블루 PC로 시작 신호를 보낸다"의 실체다 — 새 토픽을 따로 만드는 게 아니라,
frontend-judge-pc/team-pc 양쪽이 이미 구독하고 있는 /judge/match_state에 "RUNNING"을
publish하면 두 팀 PC 화면(MatchStateBanner)에 바로 반영된다. 각 팀의 AMR이 실제로 "시작"
동작을 하려면, 그쪽 로봇 제어 노드가 이 토픽을 구독해서 RUNNING을 감지해야 한다 — 그건 이
백엔드가 아니라 로봇/팀 PC 쪽 코드의 몫이다.

실행:
    ros2 run rosbridge_server rosbridge_websocket &   # 프론트엔드가 붙는 rosbridge (별도 설치 필요)
    uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from .match_store import Match, MatchStore
from .ros_bridge import RosBridge

store = MatchStore()
bridge = RosBridge()


@asynccontextmanager
async def lifespan(app: FastAPI):
    bridge.start(on_hit=store.record_hit)
    yield
    bridge.stop()


app = FastAPI(title="turtlebot4_wargame judge-pc backend", lifespan=lifespan)

# 로컬 LAN 안에서 팀 PC/심판 PC 브라우저가 자유롭게 붙어야 하는 환경(폐쇄망 대회장)이라
# 오리진을 넓게 허용한다 — 인터넷에 노출되는 배포라면 프론트엔드 origin으로 좁혀야 한다.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class MatchSummary(BaseModel):
    id: int
    started_at: str
    ended_at: Optional[str]
    status: str
    winner: Optional[str]


class HitEventOut(BaseModel):
    id: int
    timestamp: str
    victim_color: str
    blue_hp_after: int
    red_hp_after: int


class MatchDetail(MatchSummary):
    hits: list[HitEventOut]


def _to_summary(m: Match) -> MatchSummary:
    return MatchSummary(id=m.id, started_at=m.started_at, ended_at=m.ended_at, status=m.status, winner=m.winner)


def _to_detail(m: Match) -> MatchDetail:
    return MatchDetail(
        id=m.id, started_at=m.started_at, ended_at=m.ended_at, status=m.status, winner=m.winner,
        hits=[HitEventOut(**vars(h)) for h in m.hits],
    )


@app.post("/api/match/start")
def start_match():
    try:
        match = store.start_match()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    bridge.node.reset_hp()  # 새 경기 시작 -> 양팀 HP 3/3으로 리셋 후 /red|blue/status publish
    bridge.node.publish_match_state("RUNNING")
    return {"match_id": match.id, "match_state": "RUNNING"}


@app.post("/api/match/end")
def end_match():
    hp = bridge.node.hp
    try:
        match = store.end_match(blue_hp=hp["blue"], red_hp=hp["red"])
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    bridge.node.publish_match_state("END")
    return {"match_id": match.id, "match_state": "END", "winner": match.winner}


@app.get("/api/matches", response_model=list[MatchSummary])
def list_matches():
    return [_to_summary(m) for m in store.list_matches()]


@app.get("/api/matches/{match_id}", response_model=MatchDetail)
def get_match(match_id: int):
    match = store.get_match(match_id)
    if match is None:
        raise HTTPException(status_code=404, detail="경기를 찾을 수 없습니다")
    return _to_detail(match)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
