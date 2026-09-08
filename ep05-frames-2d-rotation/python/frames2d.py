"""프레임 · 2D 회전 · 강체 변환 — 예제 답안 (Python)

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.5

- rot2d(deg)                   : 2×2 회전행렬 R(θ) = [[cosθ, −sinθ], [sinθ, cosθ]]  (열 = 돌아간 x축·y축)
- rigid_2d(points, deg, tx, ty): 강체 변환 p' = R p + t  (회전 후 이동, 점 목록 전체)
- to_base(deg, t, p)           : 툴 프레임 좌표 → 베이스 프레임 좌표
- to_tool(deg, t, p)           : 베이스 프레임 좌표 → 툴 프레임 좌표 (이동량 빼고 Rᵀ 곱하기)
- orth_check(deg)              : RᵀR 을 계산해 항등행렬인지 확인 (직교성)
- wrap(a) / wrap_deg(d)        : 각도를 [−π, π] / [−180°, 180°] 로 접어 넣기 (래핑 = 각도 정규화)
- ang_err(target, current)     : 최단 각오차 = wrap(target − current)
- circ_mean_deg(degs)          : 원형 평균 = atan2(Σ sinθ, Σ cosθ)

실행: python frames2d.py  →  콘솔 출력 + points.csv (x,y,xr,yr) + angles.txt
"""
import csv
import math

PI = math.pi


# ── 회전 · 강체 변환 ────────────────────────────────────────────────────────
def rot2d(deg):
    """도(degree) 단위 각도로 2×2 회전행렬을 만든다. 코드의 sin/cos 는 라디안만 받는다."""
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return ((c, -s), (s, c))            # 첫째 열 (c, s) = (1,0)이 돌아간 자리, 둘째 열 (−s, c) = (0,1)이 돌아간 자리


def apply(R, p):
    """행렬 R 에 점 p 를 곱한다 (첫째 열 × x + 둘째 열 × y)."""
    (a, b), (c, d) = R
    return (a * p[0] + b * p[1], c * p[0] + d * p[1])


def rigid_2d(points, deg, tx, ty):
    """강체 변환: 각 점을 원점 기준 deg 만큼 반시계 회전한 뒤 (tx, ty) 만큼 이동한다. 순서 = 회전 → 이동."""
    R = rot2d(deg)
    out = []
    for p in points:
        x, y = apply(R, p)
        out.append((x + tx, y + ty))
    return out


def to_base(deg, t, p):
    """툴 프레임에서 읽은 점 p 를 베이스 프레임 좌표로. 툴 자세 = (회전 deg, 이동 t)."""
    return rigid_2d([p], deg, t[0], t[1])[0]


def to_tool(deg, t, p):
    """베이스 프레임에서 읽은 점 p 를 툴 프레임 좌표로. 이동량을 먼저 빼고 전치행렬 Rᵀ 를 곱한다(직교성)."""
    (a, b), (c, d) = rot2d(deg)
    dx, dy = p[0] - t[0], p[1] - t[1]
    return (a * dx + c * dy, b * dx + d * dy)      # Rᵀ = [[a, c], [b, d]]


def orth_check(deg):
    """RᵀR 을 계산해 돌려준다. 회전행렬이면 항등행렬 [[1,0],[0,1]] 이 나온다."""
    (a, b), (c, d) = rot2d(deg)
    rtr = ((a * a + c * c, a * b + c * d),
           (b * a + d * c, b * b + d * d))
    max_err = max(abs(rtr[0][0] - 1), abs(rtr[0][1]), abs(rtr[1][0]), abs(rtr[1][1] - 1))
    return rtr, max_err


def det2(deg):
    (a, b), (c, d) = rot2d(deg)
    return a * d - b * c


# ── 각도 도구 셋 ─────────────────────────────────────────────────────────────
def wrap(a):
    """라디안 각도를 [−π, π] 로 접어 넣는다. ((a + π) mod 2π) − π  — Python 의 % 는 항상 0 이상을 돌려준다."""
    return (a + PI) % (2 * PI) - PI


def wrap_deg(d):
    """도 단위 래핑 [−180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


def ang_err(target, current):
    """최단 각오차(라디안): 현재에서 목표까지 가장 짧게 도는 각. 단순 뺄셈만 하면 한 바퀴를 헛돈다."""
    return wrap(target - current)


def circ_mean_deg(degs):
    """원형 평균(도): 각도를 단위 화살표로 바꿔 더한 뒤 그 방향을 atan2 로 읽는다. (y 먼저, x 나중)"""
    s = sum(math.sin(math.radians(d)) for d in degs)
    c = sum(math.cos(math.radians(d)) for d in degs)
    return math.degrees(math.atan2(s, c))


# ── 파일 출력 ────────────────────────────────────────────────────────────────
def save_points(path, points, moved):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "xr", "yr"])
        for (x, y), (xr, yr) in zip(points, moved):
            w.writerow([f"{x:.4f}", f"{y:.4f}", f"{xr:.4f}", f"{yr:.4f}"])


if __name__ == "__main__":
    # 문제 #226 — 한 변 1 인 정사각형, 30°, t = (1, 0.5)
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    deg, t = 30.0, (1.0, 0.5)
    moved = rigid_2d(square, deg, t[0], t[1])
    R = rot2d(deg)
    print("R(30) =", f"[[{R[0][0]:.4f}, {R[0][1]:.4f}], [{R[1][0]:.4f}, {R[1][1]:.4f}]]")
    print("square 30deg t=(1,0.5):", " ".join(f"({x:.4f}, {y:.4f})" for x, y in moved))
    save_points("points.csv", square, moved)

    # 프레임 왕복 — 툴 자세 = R(30°), t = (1, 0.5)
    pb = to_base(deg, t, (1.0, 0.0))
    pt = to_tool(deg, t, (2.0, 1.0))
    back = to_base(deg, t, pt)
    print(f"tool(1,0)->base({pb[0]:.4f}, {pb[1]:.4f})")
    print(f"base(2,1)->tool({pt[0]:.4f}, {pt[1]:.4f})")
    print(f"->back({back[0]:.4f}, {back[1]:.4f})")

    # 순서 오해 — 먼저 밀고 나서 돌리면 다른 자리
    x, y = apply(R, (1.0 + t[0], 0.0 + t[1]))
    print(f"move-then-rotate (1,0): ({x:.4f}, {y:.4f})   (rotate-then-move: ({moved[1][0]:.4f}, {moved[1][1]:.4f}))")

    # 직교성 — RᵀR = I, det = 1, trace = 2cosθ
    rtr, err = orth_check(deg)
    print(f"RtR = [[{rtr[0][0]:.4f}, {rtr[0][1]:.4f}], [{rtr[1][0]:.4f}, {rtr[1][1]:.4f}]]  max_err={err:.1e}")
    print(f"det = {det2(deg):.4f}   trace = {R[0][0] + R[1][1]:.4f}")

    # 각도 도구 셋
    lines = [
        f"ang_err(3.0,-3.0)={ang_err(3.0, -3.0):.4f}",
        f"ang_err(0.1,-0.1)={ang_err(0.1, -0.1):.4f}",
        f"circ_mean([170,-170])={circ_mean_deg([170, -170]):.4f}",
        f"circ_mean([10,-10,350])={circ_mean_deg([10, -10, 350]):.4f}",
        f"wrap(359deg)={wrap_deg(359.0):.4f}deg",
        f"orth_check(30)={'identity(max_err<1e-12)' if err < 1e-12 else 'FAIL'}",
    ]
    for ln in lines:
        print(ln)
    print(f"(naive) 3.0-(-3.0)={3.0 - (-3.0):.4f}   mean([170,-170])={sum([170, -170]) / 2:.4f}   mean([10,-10,350])={sum([10, -10, 350]) / 3:.4f}")
    with open("angles.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("saved: points.csv, angles.txt")
