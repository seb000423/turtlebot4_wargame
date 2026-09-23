from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from ..db import crud, models
from ..db.database import get_db

router = APIRouter(prefix="/api/matches", tags=["matches"])


def _serialize_hit(hit: models.HitEvent) -> dict:
    return {
        "id": hit.id,
        "timestamp": hit.timestamp.isoformat(),
        "victim_color": hit.victim_color,
        "blue_hp_after": hit.blue_hp_after,
        "red_hp_after": hit.red_hp_after,
    }


def _serialize_match(match: models.Match, include_hits: bool = False) -> dict:
    out = {
        "id": match.id,
        "started_at": match.started_at.isoformat(),
        "ended_at": match.ended_at.isoformat() if match.ended_at else None,
        "status": match.status,
        "winner": match.winner,
    }
    if include_hits:
        out["hits"] = [_serialize_hit(h) for h in match.hits]
    return out


@router.get("")
def list_matches(db: Session = Depends(get_db)) -> list[dict]:
    """입력: 없음. 출력: 전체 경기 목록(최신순), hits는 미포함(가벼운 리스트 뷰)."""
    return [_serialize_match(m) for m in crud.list_matches(db)]


@router.get("/{match_id}")
def get_match(match_id: int, db: Session = Depends(get_db)) -> dict:
    """입력: match_id(path). 출력: 경기 상세 + hits(명중 로그 전체, HP 스냅샷 포함), 없으면 404."""
    match = crud.get_match(db, match_id)
    if match is None:
        raise HTTPException(status_code=404, detail="match not found")
    return _serialize_match(match, include_hits=True)
