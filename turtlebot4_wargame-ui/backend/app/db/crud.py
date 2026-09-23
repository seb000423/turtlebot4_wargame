from __future__ import annotations

from datetime import datetime

from sqlalchemy import select
from sqlalchemy.orm import Session

from . import models


def create_match(db: Session) -> models.Match:
    """입력: db 세션. 출력: status="RUNNING"으로 새로 생성된 Match row (id/started_at 자동 채워짐)."""
    match = models.Match(status="RUNNING")
    db.add(match)
    db.commit()
    db.refresh(match)
    return match


def end_match(db: Session, match_id: int, winner: str | None) -> models.Match | None:
    """입력: match_id, winner("blue"/"red"/None). 출력: status="END"+ended_at+winner 기록된 Match, 없으면 None."""
    match = db.get(models.Match, match_id)
    if match is None:
        return None
    match.status = "END"
    match.ended_at = datetime.utcnow()
    match.winner = winner
    db.commit()
    db.refresh(match)
    return match


def add_hit_event(
    db: Session, match_id: int, victim_color: str, blue_hp_after: int, red_hp_after: int
) -> models.HitEvent:
    """입력: match_id, victim_color, 명중 직후 양팀 HP 스냅샷. 출력: 생성된 HitEvent row(timestamp 자동)."""
    hit = models.HitEvent(
        match_id=match_id,
        victim_color=victim_color,
        blue_hp_after=blue_hp_after,
        red_hp_after=red_hp_after,
    )
    db.add(hit)
    db.commit()
    db.refresh(hit)
    return hit


def list_matches(db: Session) -> list[models.Match]:
    stmt = select(models.Match).order_by(models.Match.started_at.desc())
    return list(db.scalars(stmt))


def get_match(db: Session, match_id: int) -> models.Match | None:
    return db.get(models.Match, match_id)
