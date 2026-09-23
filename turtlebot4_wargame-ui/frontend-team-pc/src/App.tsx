import { useState } from "react";
import { AimPanel } from "./components/AimPanel";
import { CameraFeed } from "./components/CameraFeed";
import { LoginScreen } from "./components/LoginScreen";
import { MatchStateBanner } from "./components/MatchStateBanner";
import { StatusPanel } from "./components/StatusPanel";
import { useRos } from "./lib/ros";
import type { TeamColor } from "./lib/topics";

const AUTH_KEY = "turtlebot4_wargame_team_auth";

function getQueryColor(): TeamColor {
  const param = new URLSearchParams(window.location.search).get("color");
  return param === "red" ? "red" : "blue";
}

function loadStoredColor(): TeamColor | null {
  const raw = localStorage.getItem(AUTH_KEY);
  return raw === "blue" || raw === "red" ? raw : null;
}

/**
 * 입력: 없음(props X) — URL의 ?color= 쿼리(로그인 화면 초기 탭 선택용)와 localStorage(로그인 유지)만 읽음.
 * 출력: 없음(최상위 컴포넌트) — 로그인 전엔 LoginScreen, 로그인 후엔 배너+카메라+상태+조준 대시보드를 렌더.
 */
export default function App() {
  const [color, setColor] = useState<TeamColor | null>(loadStoredColor);
  const { ros, status } = useRos();

  if (!color) {
    return (
      <LoginScreen
        initialColor={getQueryColor()}
        onLogin={(c) => {
          localStorage.setItem(AUTH_KEY, c);
          setColor(c);
        }}
      />
    );
  }

  return (
    <div className={`app app--${color}`}>
      <MatchStateBanner
        ros={ros}
        color={color}
        connStatus={status}
        onLogout={() => {
          localStorage.removeItem(AUTH_KEY);
          setColor(null);
        }}
      />
      <main className="grid">
        <CameraFeed ros={ros} color={color} />
        <div className="grid__side">
          <StatusPanel ros={ros} color={color} />
          <AimPanel ros={ros} color={color} />
        </div>
      </main>
    </div>
  );
}
