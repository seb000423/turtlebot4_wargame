# 팀 로봇 1대의 상태 요약. UI의 /{color}/status 용.
# mission_state 값은 backend/app/config.py의 MISSION_STATES 어휘를 따른다:
#   IDLE | CHASE | AIM | WAIT_APPROVAL | FIRE | RETURN | DISABLED
int32 self_hp
float32 battery
int32 ammo                # 남은 장전 탄수 (0 ~ MAX_AMMO, UI 쪽 MAX_AMMO=5 와 일치시킬 것)
float32 reload_remaining  # 재장전 남은 초. 0이면 발사 가능, 0보다 크면 재장전 중
string mission_state
bool robot_destroyed
float64 timestamp
