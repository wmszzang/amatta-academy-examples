"""EP.20: 일곱 연습문제의 이동로봇 기구학과 검산."""
import csv
import math
from pathlib import Path


def diff_ik(v, w, r, L):
    vL, vR = v - w * L / 2, v + w * L / 2
    return vL, vR, vL / r, vR / r


def diff_fk(vL, vR, L):
    return (vL + vR) / 2, (vR - vL) / L


def mecanum_ik(vx, vy, w, r, lx, ly):
    k = lx + ly
    return ((vx - vy - k*w)/r, (vx + vy + k*w)/r,
            (vx + vy - k*w)/r, (vx - vy + k*w)/r)


def mecanum_fk(FL, FR, RL, RR, r, lx, ly):
    return (r*(FL+FR+RL+RR)/4, r*(-FL+FR+RL-RR)/4,
            r*(-FL+FR-RL+RR)/(4*(lx+ly)))


def ackermann(L, T, R):
    # 본 문제는 양의 축거와 R > T/2인 좌선회를 다룬다.
    if L <= 0 or T < 0 or R <= T/2:
        raise ValueError("Require L > 0, T >= 0, R > T/2")
    return (math.degrees(math.atan(L/(R-T/2))),
            math.degrees(math.atan(L/(R+T/2))))


def bicycle_step(x, y, th, v, d, L, dt):
    # 세 결과를 모두 갱신 전 방향각으로 계산한다.
    return (x+v*math.cos(th)*dt, y+v*math.sin(th)*dt,
            th+(v/L)*math.tan(d)*dt)


def odometry(dL, dR, L, N, midpoint=True):
    x = y = th = 0.0
    pts = [(x, y)]
    dc, dth = (dL+dR)/2, (dR-dL)/L
    for _ in range(N):
        angle = th+dth/2 if midpoint else th
        x += dc*math.cos(angle)
        y += dc*math.sin(angle)
        th += dth
        pts.append((x, y))
    return pts, th


def tick_to_dist(ticks, r, CPR, dt):
    per, cumulative = 2*math.pi*r/CPR, 0.0
    for increment in ticks:
        cumulative += increment*per
        yield cumulative, increment*per/dt


def pure_pursuit(gy, Ld, L):
    if Ld <= 0:
        raise ValueError("Look-ahead distance must be positive")
    kappa = 2*gy/(Ld*Ld)
    # 제출값은 곡률과 조향각이다. 직진에서 반경의 역수를 계산하지 않는다.
    return kappa, math.atan(kappa*L)


def results():
    wheels = diff_ik(.5, .4, .05, .3)
    four = mecanum_ik(1, .5, .2, .05, .3, .25)
    pose = (0.0, 0.0, 0.0)
    for _ in range(10):
        pose = bicycle_step(*pose, 1, .1, 2, .1)
    pts, th = odometry(.1, .12, .5, 40)
    rows = [("261", *wheels), ("261_fk", *diff_fk(*wheels[:2], .3)),
            ("483", *four), ("483_fk", *mecanum_fk(*four, .05, .3, .25)),
            ("481", *ackermann(2.5, 1.5, 10)), ("463", *pose),
            ("224", *pts[-1], th), ("254", *pure_pursuit(1, 5, 2.5))]
    rows.extend((f"230_{i}", *p) for i, p in enumerate(tick_to_dist([0,100,205,300,412], .05, 2048, .1)))
    exact = (2.75*math.sin(1.6), 2.75*(1-math.cos(1.6)))
    old, _ = odometry(.1, .12, .5, 40, midpoint=False)
    slip, sth = odometry(.1, .12*.97, .5, 40)
    rows.extend([("exact", *exact), ("start_angle", *old[-1], math.dist(old[-1], exact)),
                 ("slip", *slip[-1], math.dist(slip[-1], pts[-1]), math.degrees(sth-th))])
    return rows, pts


def verify():
    rows, pts = results()
    expected = {"261": (.4400,.5600,8.8000,11.2000), "483": (7.8000,32.2000,27.8000,12.2000),
                "481": (15.1240,13.0919), "463": (.9996,.0226,.0502),
                "224": (2.7490,2.8305,1.6000), "254": (.0800,.1974)}
    for key, *values in rows:
        if key in expected:
            assert all(abs(a-b) < .00005 for a,b in zip(values, expected[key])), key
    assert len(pts) == 41 and pts[0] == (0,0)
    assert pure_pursuit(0,5,2.5) == (0,0)
    assert bicycle_step(0,0,0,1,0,2,.1) == (.1,0,0)
    assert odometry(.1,.1,.5,2)[0][-1] == (.2,0)
    try:
        pure_pursuit(1,0,2.5)
    except ValueError:
        pass
    else:
        raise AssertionError("Zero look-ahead must fail")
    return rows, pts


if __name__ == "__main__":
    rows, pts = verify()
    for key, *values in rows:
        print(key+","+",".join(f"{v:.4f}" for v in values))
    with Path("trajectory-python.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["step","x_m","y_m"])
        writer.writerows((i, f"{x:.4f}", f"{y:.4f}") for i,(x,y) in enumerate(pts))
