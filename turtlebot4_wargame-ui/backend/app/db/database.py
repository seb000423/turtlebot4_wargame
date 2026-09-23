from __future__ import annotations

from pathlib import Path
from typing import Iterator

from sqlalchemy import create_engine
from sqlalchemy.orm import DeclarativeBase, Session, sessionmaker

DB_PATH = Path(__file__).resolve().parent.parent.parent / "turtlebot4_wargame.db"
engine = create_engine(f"sqlite:///{DB_PATH}", connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(bind=engine, autoflush=False, autocommit=False)


class Base(DeclarativeBase):
    pass


def init_db() -> None:
    from . import models  # noqa: F401  모델 등록을 위해 import

    Base.metadata.create_all(bind=engine)


def get_session() -> Session:
    """입력 없음. 출력: 새 Session — 호출자가 직접 close()해야 함(mock/generators.py에서 사용)."""
    return SessionLocal()


def get_db() -> Iterator[Session]:
    """입력 없음. 출력: FastAPI Depends용 제너레이터 — 요청 끝나면 자동 close (api/*.py에서 사용)."""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
