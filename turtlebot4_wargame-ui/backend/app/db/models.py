from __future__ import annotations

from datetime import datetime

from sqlalchemy import DateTime, ForeignKey, Integer, String
from sqlalchemy.orm import Mapped, mapped_column, relationship

from .database import Base


class Match(Base):
    """경기 1건. control.py의 start_match/end_match가 쓰고, matches.py가 읽는다."""

    __tablename__ = "matches"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    started_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    ended_at: Mapped[datetime | None] = mapped_column(DateTime, nullable=True)
    status: Mapped[str] = mapped_column(String(16), default="RUNNING")  # RUNNING | END
    winner: Mapped[str | None] = mapped_column(String(8), nullable=True)  # blue | red | None(무승부)

    hits: Mapped[list["HitEvent"]] = relationship(back_populates="match", cascade="all, delete-orphan")


class HitEvent(Base):
    """명중 1건. hit_event_loop(mock/generators.py)가 쓰고, matches.py 상세 조회가 읽는다."""

    __tablename__ = "hit_events"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    match_id: Mapped[int] = mapped_column(ForeignKey("matches.id"))
    timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    victim_color: Mapped[str] = mapped_column(String(8))
    blue_hp_after: Mapped[int] = mapped_column(Integer)
    red_hp_after: Mapped[int] = mapped_column(Integer)

    match: Mapped["Match"] = relationship(back_populates="hits")
