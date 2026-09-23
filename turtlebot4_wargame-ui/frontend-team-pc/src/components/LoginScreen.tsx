import { useState, type FormEvent } from "react";
import type { TeamColor } from "../lib/topics";

// 백엔드에 유저/인증 시스템이 없어서 클라이언트에서만 검증하는 데모용 게이트.
const CREDENTIALS: Record<TeamColor, string> = {
  blue: "blue1234",
  red: "red1234",
};

/**
 * 입력: initialColor(로그인 화면 진입 시 미리 선택해둘 팀, ?color= 쿼리에서 옴).
 * 출력: onLogin(color) 콜백 — 비밀번호가 CREDENTIALS[color]와 일치할 때만 호출됨(부모가 localStorage 저장 담당).
 */
export function LoginScreen({
  initialColor,
  onLogin,
}: {
  initialColor: TeamColor;
  onLogin: (color: TeamColor) => void;
}) {
  const [color, setColor] = useState<TeamColor>(initialColor);
  const [password, setPassword] = useState("");
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = (e: FormEvent) => {
    e.preventDefault();
    if (password !== CREDENTIALS[color]) {
      setError("비밀번호가 올바르지 않습니다.");
      return;
    }
    onLogin(color);
  };

  return (
    <div className="login">
      <form className="login__card" onSubmit={handleSubmit}>
        <h1 className="login__title">TurtleBot4 Wargame 팀 PC</h1>
        <p className="login__subtitle">소속 팀을 선택하고 로그인하세요</p>

        <div className="login__tabs">
          <button
            type="button"
            className={`login__tab login__tab--blue ${color === "blue" ? "login__tab--active" : ""}`}
            onClick={() => setColor("blue")}
          >
            BLUE
          </button>
          <button
            type="button"
            className={`login__tab login__tab--red ${color === "red" ? "login__tab--active" : ""}`}
            onClick={() => setColor("red")}
          >
            RED
          </button>
        </div>

        <input
          className="login__input"
          type="password"
          placeholder="비밀번호"
          value={password}
          onChange={(e) => {
            setPassword(e.target.value);
            setError(null);
          }}
          autoFocus
        />

        {error && <p className="login__error">{error}</p>}

        <button type="submit" className="login__submit">
          로그인
        </button>
      </form>
    </div>
  );
}
