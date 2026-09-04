"""3차·5차 다항식 관절 궤적 — 부드럽게 출발해 부드럽게 멈추는 점대점 이동.

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3

실행:  python polynomial_traj.py
출력:  cubic.csv, quintic.csv  (열: t, q, qd, qdd = 시간, 각도, 각속도, 각가속도)

예제 조건(영상과 동일): 시작 각도 0도 → 목표 각도 90도, 총 시간 2초
  - 3차: 계수 a2 = 3·90/2² = 67.5, a3 = -2·90/2³ = -22.5
         → 1초(중간)에서 각도 45도, 최고 각속도 초당 67.5도, 가속도 135 → -135 (출발·도착 순간 점프)
  - 5차: q = 90·(10τ³ - 15τ⁴ + 6τ⁵), τ = t/T
         → 1초에서 45도, 최고 각속도 약 초당 84.4도, 최고 각가속도 약 130 (가속도까지 0에서 시작·0으로 끝남)
"""
import csv


def cubic(q0, qf, T, dt=0.01):
    d = qf - q0
    a2, a3 = 3 * d / T**2, -2 * d / T**3
    rows, t = [], 0.0
    while t <= T + 1e-9:
        q = q0 + a2 * t**2 + a3 * t**3
        qd = 2 * a2 * t + 3 * a3 * t**2
        qdd = 2 * a2 + 6 * a3 * t
        rows.append((t, q, qd, qdd))
        t += dt
    return rows


def quintic(q0, qf, T, dt=0.01):
    d = qf - q0
    rows, t = [], 0.0
    while t <= T + 1e-9:
        s = t / T
        q = q0 + d * (10 * s**3 - 15 * s**4 + 6 * s**5)
        qd = d * (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        qdd = d * (60 * s - 180 * s**2 + 120 * s**3) / T**2
        rows.append((t, q, qd, qdd))
        t += dt
    return rows


def save(rows, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "q", "qd", "qdd"])
        for t, q, qd, qdd in rows:
            w.writerow([f"{t:.3f}", f"{q:.4f}", f"{qd:.4f}", f"{qdd:.4f}"])


if __name__ == "__main__":
    Q0, QF, T = 0.0, 90.0, 2.0          # 시작 각도(도), 목표 각도(도), 총 시간(초)
    for name, fn in (("cubic", cubic), ("quintic", quintic)):
        rows = fn(Q0, QF, T)
        save(rows, f"{name}.csv")
        mid = min(rows, key=lambda r: abs(r[0] - T / 2))
        vmax = max(r[2] for r in rows)
        amax = max(abs(r[3]) for r in rows)
        print(f"{name:8s} 중간({mid[0]:.2f}초) 각도 = {mid[1]:.2f}도 | "
              f"최고 각속도 = {vmax:.3f} | 최고 각가속도 = {amax:.1f} | "
              f"끝 각도 = {rows[-1][1]:.2f}도 · 끝 속도 = {rows[-1][2]:.4f}")
    print("저장: cubic.csv, quintic.csv")
