"""사다리꼴 속도 프로파일 운동계획 — Python 예제 답안.

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.2

실행:
    python trapezoid_profile.py
→ 콘솔에 요약을 출력하고, 같은 폴더에 profile.csv(시간,거리,속도)를 저장합니다.
그래프는 visualize.py로 그립니다(README 참고).
"""


def profile(L, V, A, dt=0.01):
    """목표거리 L, 최고속도 V, 가속도 A로 (시간, 거리, 속도) 표를 만든다.

    구간: 가속(+A) → 등속(V) → 감속(-A).
    가속·감속에 필요한 거리(2*D)가 목표 L 이상이면 최고 속도에 닿지 못하는
    삼각형 프로파일이 되고, 꼭짓점 속도는 sqrt(A*L)이 된다.
    """
    D = V * V / (2 * A)            # 가속에 필요한 거리
    if 2 * D >= L:                 # 목표가 짧으면: 삼각형
        V = (A * L) ** 0.5
    t1 = V / A                     # 가속이 끝나는 시각
    tc = max(L - V * V / A, 0) / V # 등속 구간 시간
    T = 2 * t1 + tc                # 총 시간
    rows, s, t = [], 0.0, 0.0
    while t <= T:
        if t < t1:        v = A * t
        elif t < t1 + tc: v = V
        else:             v = max(V - A * (t - t1 - tc), 0)
        s += v * dt
        rows.append((t, s, v))
        t += dt
    return rows


if __name__ == "__main__":
    # 영상 예제 그대로: 목표 180도 · 최고 속도 초당 90도 · 가속도 매초 초당 180도
    rows = profile(L=180, V=90, A=180)
    with open("profile.csv", "w", encoding="utf-8") as f:
        f.write("t,s,v\n")
        for t, s, v in rows:
            f.write(f"{t:.3f},{s:.4f},{v:.4f}\n")
    print(f"총 시간 = {rows[-1][0]:.2f}초, 최종 거리 = {rows[-1][1]:.1f}도  →  profile.csv 저장 완료")
