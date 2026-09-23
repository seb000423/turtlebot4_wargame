"""
경기 기록(진행중/기록된 경기 목록) 저장소. 지금은 프로세스 메모리에만 들고 있다 — 백엔드가
재시작되면 기록이 날아간다. 여러 경기를 영구 보관해야 하면 SQLite로 바꾸면 되는데, 지금은
REST 응답 스키마(frontend-judge-pc/src/lib/api.ts의 MatchSummary/MatchDetail)만 맞추면
충분해서 dataclass + dict로 최소 구현.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from itertools import count

from .config import STATE_END, STATE_RUNNING


@dataclass
class HitEventRecord:
    id: int
    timestamp: str
    victim_color: str
    blue_hp_after: int
    red_hp_after: int


@dataclass
class Match:
    id: int
    started_at: str
    ended_at: str | None = None
    status: str = STATE_RUNNING
    winner: str | None = None
    hits: list[HitEventRecord] = field(default_factory=list)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class MatchStore:
    def __init__(self) -> None:
        self._matches: dict[int, Match] = {}
        self._id_counter = count(1)
        self._current: Match | None = None

    @property
    def current_state(self) -> str:
        return STATE_RUNNING if self._current is not None else STATE_END

    def start_match(self) -> Match:
        if self._current is not None:
            raise ValueError("경기가 이미 진행 중입니다")
        match = Match(id=next(self._id_counter), started_at=_now_iso())
        self._matches[match.id] = match
        self._current = match
        return match

    def end_match(self, blue_hp: int, red_hp: int) -> Match:
        if self._current is None:
            raise ValueError("진행 중인 경기가 없습니다")
        match = self._current
        match.ended_at = _now_iso()
        match.status = STATE_END
        if blue_hp > red_hp:
            match.winner = "blue"
        elif red_hp > blue_hp:
            match.winner = "red"
        else:
            match.winner = None  # 동점 무승부
        self._current = None
        return match

    def record_hit(self, victim_color: str, blue_hp_after: int, red_hp_after: int) -> None:
        """JudgeRosNode가 /judge/hit_event를 받아 HP를 깎을 때마다 호출 — 진행 중인 경기가
        있으면 그 경기의 hits 리스트에 기록한다. 진행 중인 경기가 없으면(경기 시작 전 오탐 등)
        조용히 무시한다."""
        if self._current is None:
            return
        record = HitEventRecord(
            id=len(self._current.hits) + 1,
            timestamp=_now_iso(),
            victim_color=victim_color,
            blue_hp_after=blue_hp_after,
            red_hp_after=red_hp_after,
        )
        self._current.hits.append(record)

    def list_matches(self) -> list[Match]:
        return sorted(self._matches.values(), key=lambda m: m.id, reverse=True)

    def get_match(self, match_id: int) -> Match | None:
        return self._matches.get(match_id)
