"""S-커브(저크 제한) 속도 프로파일 — 사다리꼴의 모서리를 둥글게.

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3

실행:  python scurve.py
출력:  scurve.csv  (열: t, x, v, a, j = 시간, 위치, 속도, 가속도, 저크)

예제 조건(영상과 동일): 목표 90도, 최고 속도 초당 60도, 최고 가속도 120, 저크 한계 480
  - 저크 구간 tj = 120/480 = 0.25초 → 가속 구간 0.75초(저크+ · 가속 일정 · 저크-)
  - 가속 구간 거리 = 60 × 0.75 / 2 = 22.5도 (감속도 대칭 22.5도) → 등속 45도 = 0.75초
  - 총 시간 2.25초 (같은 조건의 사다리꼴 2.0초보다 저크 구간 하나만큼 길다)

핵심: 7구간마다 저크를 +J / 0 / -J 로 정해 두고, 시간을 잘게 쪼개며
      가속도 → 속도 → 위치를 차례로 누적(적분)한다. 저크가 구간 안에서 일정하므로
      아래 세 줄 갱신식은 dt 크기와 무관하게 정확하다.
"""
import csv


def scurve(L, V, A, J, dt=0.001):
    tj = A / J                          # 저크 구간 길이
    if V / A < tj:                      # 최고 가속도에 못 미치는 짧은 경우
        tj = (V / J) ** 0.5
        A = J * tj
    ta = V / A + tj                     # 가속 구간 전체(저크+ · 일정 · 저크-)
    da = V * ta / 2                     # 가속 구간 거리
    tc = (L - 2 * da) / V               # 등속 시간 (음수면 목표가 너무 짧은 경우 — 이 예제 범위 밖)
    segs = [(tj, +J), (ta - 2 * tj, 0.0), (tj, -J), (tc, 0.0),
            (tj, -J), (ta - 2 * tj, 0.0), (tj, +J)]
    rows, t, x, v, a = [(0.0, 0.0, 0.0, 0.0, +J)], 0.0, 0.0, 0.0, 0.0
    for dur, j in segs:
        for _ in range(int(round(dur / dt))):
            x += v * dt + a * dt**2 / 2 + j * dt**3 / 6
            v += a * dt + j * dt**2 / 2
            a += j * dt
            t += dt
            rows.append((t, x, v, a, j))
    return rows


if __name__ == "__main__":
    rows = scurve(L=90.0, V=60.0, A=120.0, J=480.0)
    with open("scurve.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "v", "a", "j"])
        for r in rows:
            w.writerow([f"{r[0]:.3f}", f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.4f}", f"{r[4]:.1f}"])
    print(f"총 시간 = {rows[-1][0]:.2f}초, 최종 위치 = {rows[-1][1]:.2f}도, "
          f"최고 속도 = {max(r[2] for r in rows):.2f}, 최고 가속도 = {max(r[3] for r in rows):.2f}")
    print("저장: scurve.csv")
