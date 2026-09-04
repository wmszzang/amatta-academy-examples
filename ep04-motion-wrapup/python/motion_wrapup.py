"""운동계획 끝내기 - 지문을 읽고 계획을 고른 뒤, 네 지문을 끝까지 푼다.

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.4

실행:  python motion_wrapup.py
출력:  p1_trapezoid.csv (t, x, v, a)      지문 ① 사다리꼴   - 목표 150도 · 최고 속도 60 · 최고 가속도 120
       p2_cubic.csv     (t, q, qd, qdd)   지문 ② 3차 다항식 - 30도 → 120도 · 3초 · 시작·끝 정지
       p3_quintic.csv   (t, q, qd, qdd)   지문 ③ 5차 다항식 - -45도 → 45도 · 4초 · 각속도·각가속도 모두 0
       p4_scurve.csv    (t, x, v, a, j)   지문 ④ S-커브     - 목표 180도 · 속도 90 · 가속도 180 · 저크 720

판별 규칙(영상 §갈림길):
  총 시간 T가 주어지면 → 다항식(시간을 맞춘다)        : 각가속도까지 0이면 5차, 아니면 3차
  한계(속도·가속도)가 주어지면 → 사다리꼴 계열(가능한 빨리) : 저크 한계까지 있으면 S-커브
"""
import csv
import math


# ── 0. 판별 ───────────────────────────────────────────────────────────────
def choose_plan(given):
    """given: 지문에서 읽어 낸 값의 사전. 예) {"D": 150, "V": 60, "A": 120}"""
    if "T" in given:                       # 총 시간이 정해져 있다 → 다항식
        return "quintic" if given.get("zero_accel") else "cubic"
    if "J" in given:                       # 저크 한계까지 있다 → S-커브
        return "scurve"
    return "trapezoid"                     # 속도·가속도 한계만 → 사다리꼴


# ── 1. 사다리꼴 (삼각형 축퇴 포함) ─────────────────────────────────────────
def trapezoid(D, V, A, dt=0.01):
    if V * V / A > D:                      # 가속+감속 거리(V²/A)가 목표보다 길면 → 삼각형
        V = math.sqrt(A * D)               # 꼭짓점 속도 = 제곱근(가속도 × 거리)
    ta = V / A                             # 가속 시간
    tc = (D - V * V / A) / V               # 등속 시간 (삼각형이면 0)
    T = 2 * ta + tc
    rows = []
    for i in range(int(round(T / dt)) + 1):
        t = i * dt
        if t < ta:                                       # 가속
            a, v, x = A, A * t, A * t * t / 2
        elif t < ta + tc:                                # 등속
            a, v, x = 0.0, V, V * ta / 2 + V * (t - ta)
        else:                                            # 감속 (남은 시간으로 거꾸로)
            r = max(T - t, 0.0)
            a, v, x = -A, A * r, D - A * r * r / 2
        rows.append((t, x, v, a))
    return rows, T, V


# ── 2. 3차 다항식 (시작·끝 속도 0) ────────────────────────────────────────
def cubic(q0, qf, T, dt=0.01):
    d = qf - q0                            # 이동량 = 목표 - 시작 (시작이 0이 아닐 수 있다!)
    a2, a3 = 3 * d / T**2, -2 * d / T**3   # 계수 두 줄
    rows = []
    for i in range(int(round(T / dt)) + 1):
        t = i * dt
        q = q0 + a2 * t**2 + a3 * t**3     # 상수항 = 시작 각도
        qd = 2 * a2 * t + 3 * a3 * t**2
        qdd = 2 * a2 + 6 * a3 * t
        rows.append((t, q, qd, qdd))
    return rows


# ── 3. 5차 다항식 (시작·끝 속도·가속도 0 - 10·15·6 공식) ─────────────────
def quintic(q0, qf, T, dt=0.01):
    d = qf - q0
    rows = []
    for i in range(int(round(T / dt)) + 1):
        t = i * dt
        s = t / T                          # 진행률 0 → 1
        q = q0 + d * (10 * s**3 - 15 * s**4 + 6 * s**5)
        qd = d * (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        qdd = d * (60 * s - 180 * s**2 + 120 * s**3) / T**2
        rows.append((t, q, qd, qdd))
    return rows


# ── 4. S-커브 (저크 제한 7구간 - 도달 못 하는 구간은 0으로) ────────────────
def scurve(L, V, A, J, dt=0.001):
    tj = A / J                             # 저크 구간 길이 (가속도가 0 → A 로 오르는 시간)
    if V / A < tj:                         # 확인 ① 가속 일정 구간이 있는가 (V ≥ A²/J)
        tj = math.sqrt(V / J)              #   없으면 저크 구간만으로 V까지 → 실제 최고 가속도가 낮아진다
        A = J * tj
    ta = V / A + tj                        # 가속 구간 전체 = 저크+ · 가속 일정 · 저크-
    da = V * ta / 2                        # 가속 구간 거리 (속도 평균 V/2 × 시간)
    if L < 2 * da:                         # 확인 ② 등속 구간이 있는가 (L ≥ 2·da)
        # 없으면 최고 속도를 낮춘다: V²/A + V·tj = L 을 V에 대해 푼다 (근의 공식)
        V = (-tj + math.sqrt(tj * tj + 4 * L / A)) * A / 2
        if V / A < tj:                     #   그래도 가속 일정 구간이 없으면 저크 구간만 남는다
            tj = (L / (2 * J)) ** (1 / 3)  #   2·J·tj³ = L
            A, V = J * tj, J * tj * tj
        ta = V / A + tj
        da = V * ta / 2
    tc = (L - 2 * da) / V                  # 등속 시간 (등속이 없으면 0)
    segs = [(tj, +J), (ta - 2 * tj, 0.0), (tj, -J), (tc, 0.0),
            (tj, -J), (ta - 2 * tj, 0.0), (tj, +J)]
    rows, t, x, v, a = [(0.0, 0.0, 0.0, 0.0, +J)], 0.0, 0.0, 0.0, 0.0
    for dur, j in segs:
        t_end = t + dur
        while t < t_end - 1e-9:
            h = min(dt, t_end - t)                            # 구간 끝은 정확히 맞춘다 (마지막 걸음만 짧게)
            x += v * h + a * h**2 / 2 + j * h**3 / 6          # 위치
            v += a * h + j * h**2 / 2                         # 속도
            a += j * h                                        # 가속도
            t += h
            rows.append((t, x, v, a, j))
    return rows, segs


def save(name, header, rows, fmt):
    with open(name, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{val:{spec}}" for val, spec in zip(r, fmt)])
    print(f"저장: {name} ({len(rows)}행)")


if __name__ == "__main__":
    # 지문 ① - 정지에서 목표 150도, 최고 속도 60, 최고 가속도 120, dt 0.01. 총 시간은?
    print("지문 ① 판별:", choose_plan({"D": 150, "V": 60, "A": 120}))
    rows, T, V = trapezoid(150, 60, 120, 0.01)
    print(f"  총 시간 {T:.2f}초 · 최고 속도 {V:.1f} · 최종 위치 {rows[-1][1]:.2f}도")
    save("p1_trapezoid.csv", ["t", "x", "v", "a"], rows, [".2f", ".4f", ".4f", ".1f"])
    rows_s, T_s, V_s = trapezoid(20, 60, 120, 0.01)      # 함정 확인: 목표 20도면 삼각형
    print(f"  (목표 20도면) 삼각형 - 꼭짓점 속도 {V_s:.2f} · 총 시간 {T_s:.3f}초")

    # 지문 ② - 30도 → 120도, 3초, 시작·끝 정지, dt 0.1
    print("지문 ② 판별:", choose_plan({"q0": 30, "qf": 120, "T": 3}))
    rows = cubic(30, 120, 3, 0.1)
    mid = rows[len(rows) // 2]
    print(f"  중간 1.5초 각도 {mid[1]:.2f} · 최고 각속도 {max(r[2] for r in rows):.2f} · "
          f"각가속도 {rows[0][3]:.1f} → {rows[-1][3]:.1f} · 끝 각도 {rows[-1][1]:.2f}")
    save("p2_cubic.csv", ["t", "q", "qd", "qdd"], rows, [".1f", ".4f", ".4f", ".4f"])

    # 지문 ③ - -45도 → 45도, 4초, 각속도·각가속도 모두 0, dt 0.2
    print("지문 ③ 판별:", choose_plan({"q0": -45, "qf": 45, "T": 4, "zero_accel": True}))
    rows = quintic(-45, 45, 4, 0.2)
    mid = rows[len(rows) // 2]
    print(f"  중간 2초 각도 {mid[1]:.2f} · 최고 각속도 {max(r[2] for r in rows):.4f} · "
          f"최고 각가속도 {max(r[3] for r in rows):.2f} · 시작 각가속도 {rows[0][3]:.1f}")
    save("p3_quintic.csv", ["t", "q", "qd", "qdd"], rows, [".1f", ".4f", ".4f", ".4f"])

    # 지문 ④ - 정지에서 목표 180도, 속도 90 · 가속도 180 · 저크 720, dt 0.005. 총 시간은?
    print("지문 ④ 판별:", choose_plan({"D": 180, "V": 90, "A": 180, "J": 720}))
    rows, segs = scurve(180, 90, 180, 720, 0.005)
    print("  7구간 길이:", [f"{d:.3f}" for d, _ in segs])
    print(f"  총 시간 {rows[-1][0]:.3f}초 · 최종 위치 {rows[-1][1]:.2f}도 · "
          f"최고 속도 {max(r[2] for r in rows):.2f} · 최고 가속도 {max(r[3] for r in rows):.2f}")
    save("p4_scurve.csv", ["t", "x", "v", "a", "j"], rows, [".3f", ".4f", ".4f", ".4f", ".1f"])
    # 확인 ②가 작동하는지: 목표를 30도로 줄이면 등속이 사라지고 최고 속도가 낮아진다
    rows_s, segs_s = scurve(30, 90, 180, 720, 0.005)
    print(f"  (목표 30도면) 등속 {segs_s[3][0]:.3f}초 · 최고 속도 {max(r[2] for r in rows_s):.2f} · "
          f"총 시간 {rows_s[-1][0]:.3f}초 · 최종 위치 {rows_s[-1][1]:.2f}도")
