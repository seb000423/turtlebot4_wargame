"""인메모리 매치 상태 — REST 컨트롤 API / 목업 생성기 / DB 기록이 공유하는 단일 소스."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from ..config import STARTING_HP, TEAM_COLORS


@dataclass
class TeamState:
    self_hp: int = STARTING_HP
    battery: float = 100.0
    mission_state: str = "IDLE"
    robot_destroyed: bool = False
    shoot_approved: bool = False  # 휴먼인더루프: 팀 PC에서 "발사 승인" 버튼을 눌러야 True가 됨


@dataclass
class MatchState:
    match_state: str = "IDLE"  # IDLE | RUNNING | END
    match_id: int | None = None
    started_at: float | None = None
    ended_at: float | None = None
    teams: dict[str, TeamState] = field(default_factory=lambda: {c: TeamState() for c in TEAM_COLORS})

    def reset_teams(self) -> None:
        for color in TEAM_COLORS:
            self.teams[color] = TeamState()

    def start(self, match_id: int) -> None:
        """입력: 새로 생성된 DB match_id. 출력 없음 — 양팀 HP/battery/mission_state를 초기값으로 리셋하고 RUNNING 진입."""
        self.reset_teams()
        self.match_state = "RUNNING"
        self.match_id = match_id
        self.started_at = time.time()
        self.ended_at = None

    def end(self) -> None:
        """입력 없음. 출력 없음 — match_state를 END로 바꾸고 ended_at을 기록(팀 HP 등은 그대로 남겨둠)."""
        self.match_state = "END"
        self.ended_at = time.time()

    def apply_hit(self, victim_color: str) -> TeamState:
        """입력: victim_color("blue"/"red"). 출력: 갱신된 TeamState — HP -1, 0이 되면 robot_destroyed=True+DISABLED."""
        team = self.teams[victim_color]
        team.self_hp = max(0, team.self_hp - 1)
        if team.self_hp == 0:
            team.robot_destroyed = True
            team.mission_state = "DISABLED"
        return team

    def winner(self) -> str | None:
        """입력 없음. 출력: 상대만 destroyed면 그 색상, 둘 다 생존/둘 다 destroyed면 None(무승부/미판정)."""
        blue_alive = not self.teams["blue"].robot_destroyed
        red_alive = not self.teams["red"].robot_destroyed
        if blue_alive and not red_alive:
            return "blue"
        if red_alive and not blue_alive:
            return "red"
        return None


state = MatchState()
