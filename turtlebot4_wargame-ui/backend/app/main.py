from __future__ import annotations

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .api import control, matches
from .db.database import init_db
from .mock.generators import start_background_tasks, stop_background_tasks
from .rosbridge.server import router as rosbridge_router

logging.basicConfig(level=logging.INFO)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """서버 기동/종료 훅. 시작 시 DB 테이블 생성 + 목업 데이터 생성기(백그라운드 asyncio task) 실행,
    종료 시 해당 task들을 정리한다. 입력 없음, 출력 없음 — FastAPI가 서버 생명주기에 맞춰 호출."""
    init_db()
    tasks = start_background_tasks()
    try:
        yield
    finally:
        await stop_background_tasks(tasks)


app = FastAPI(title="TurtleBot4 Wargame UI backend (mock)", lifespan=lifespan)

# 개발 중에는 프론트가 다른 포트(Vite)에서 뜨므로 전체 허용. 배포 시에는 origin을 좁힐 것.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(rosbridge_router)
app.include_router(matches.router)
app.include_router(control.router)


@app.get("/api/health")
def health() -> dict:
    return {"status": "ok"}
