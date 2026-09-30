"""위치 경로 예제. 내부 단위는 m, s, rad이며 출력만 반올림한다."""
import csv
import math
from pathlib import Path

L1, L2 = 1.2, 0.9
P0, P1 = (1.6, 0.3), (0.6, 1.5)


def fk2(q1, q2, l1=L1, l2=L2):
    return (l1*math.cos(q1)+l2*math.cos(q1+q2),
            l1*math.sin(q1)+l2*math.sin(q1+q2))


def ik2(x, y, l1=L1, l2=L2, elbow=1):
    if l1 <= 0 or l2 <= 0 or elbow not in (-1, 1):
        raise ValueError('positive links and elbow +/-1 required')
    radius = math.hypot(x, y)
    if radius < abs(l1-l2)-1e-12 or radius > l1+l2+1e-12:
        return None
    c = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
    # 도달성 검사 뒤 경계 부동소수 오차만 보정한다.
    q2 = elbow*math.acos(max(-1.0, min(1.0, c)))
    q1 = math.atan2(y, x)-math.atan2(l2*math.sin(q2), l1+l2*math.cos(q2))
    return q1, q2


def line_points(p0, p1, n):
    if not isinstance(n, int) or n < 1:
        raise ValueError('n must be a positive interval count')
    return [(p0[0]+(p1[0]-p0[0])*i/n,
             p0[1]+(p1[1]-p0[1])*i/n) for i in range(n+1)]


def arc_points(cx, cy, r, a0, a1, n):
    if r <= 0 or not isinstance(n, int) or n < 1:
        raise ValueError('positive radius and interval count required')
    return [(cx+r*math.cos(a0+(a1-a0)*i/n),
             cy+r*math.sin(a0+(a1-a0)*i/n)) for i in range(n+1)]


def sample_times(T, dt):
    if T <= 0 or dt <= 0:
        raise ValueError('positive time and interval required')
    times = [k*dt for k in range(math.ceil(T/dt)) if k*dt < T-1e-12]
    return times+[T]


def trap_s(t, D, T, ta):
    if D < 0 or T <= 0 or not 0 < ta <= T/2 or not 0 <= t <= T:
        raise ValueError('invalid trapezoidal profile')
    v = D/(T-ta)
    a = v/ta
    if t < ta:
        return 0.5*a*t*t
    if t < T-ta:
        return 0.5*a*ta*ta+v*(t-ta)
    return D-0.5*a*(T-t)**2


def manip(q2, l1=L1, l2=L2):
    return abs(l1*l2*math.sin(q2))


def path_ik(points, l1=L1, l2=L2, elbow=1):
    rows = []
    for x, y in points:
        q = ik2(x, y, l1, l2, elbow)
        rows.append((x, y, *(q if q else (None, None)),
                     'OK' if q else 'UNREACHABLE'))
    return rows


def joint_commands(rows):
    # 진단은 전체 실패점을 기록하지만 실행은 하나라도 실패하면 막는다.
    if any(row[4] != 'OK' for row in rows):
        raise ValueError('UNREACHABLE: no joint commands generated')
    return [(row[2], row[3]) for row in rows]


def path_ik_trap(p0, p1, T, ta, dt, l1=L1, l2=L2):
    L = math.dist(p0, p1)
    if L == 0:
        raise ValueError('path length must be positive')
    times = sample_times(T, dt)
    distances = [trap_s(t, L, T, ta) for t in times]
    points = [(p0[0]+s/L*(p1[0]-p0[0]), p0[1]+s/L*(p1[1]-p0[1]))
              for s in distances]
    diagnostic = path_ik(points, l1, l2)
    commands = joint_commands(diagnostic)
    out = []
    for i, (t, s, p, q) in enumerate(zip(times, distances, points, commands)):
        speed = (None, None) if i == 0 else tuple(
            math.degrees(q[j]-commands[i-1][j])/(t-times[i-1]) for j in (0, 1))
        out.append((t, s, *p, *map(math.degrees, q), *speed, manip(q[1], l1, l2)))
    return out


def joint_cubic(q0, q1, T, n, degree=3):
    if T <= 0 or n < 1 or degree not in (3, 5):
        raise ValueError('invalid polynomial input')
    out = []
    for k in range(n+1):
        u = k/n
        if degree == 3:
            h, dh, ddh = 3*u*u-2*u**3, 6*u-6*u*u, 6-12*u
        else:
            h = 10*u**3-15*u**4+6*u**5
            dh, ddh = 30*u*u-60*u**3+30*u**4, 60*u-180*u*u+120*u**3
        d = q1-q0
        out.append((k*T/n, q0+d*h, d*dh/T, d*ddh/(T*T)))
    return out


def deviation(p):
    dx, dy = P1[0]-P0[0], P1[1]-P0[1]
    return abs(dx*(p[1]-P0[1])-dy*(p[0]-P0[0]))/math.hypot(dx, dy)


def write_csv(filename, headers, rows):
    with open(filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(headers.split(','))
        for row in rows:
            writer.writerow('' if v is None else f'{0.0 if abs(v)<0.00005 else v:.4f}'
                            if isinstance(v, (int, float)) else v for v in row)


def compare():
    q0, q1 = ik2(*P0), ik2(*P1)
    rows = []
    for i, p in enumerate(line_points(P0, P1, 10)):
        u = i/10
        qj = tuple(a+u*(b-a) for a, b in zip(q0, q1))
        pj, qc = fk2(*qj), ik2(*p)
        rows.append((u, *map(math.degrees, qj), *pj, deviation(pj), *p, *map(math.degrees, qc)))
    write_csv('compare.csv', 'u,th1_joint,th2_joint,x_joint,y_joint,dev,x_cart,y_cart,th1_cart,th2_cart', rows)
    dense = [fk2(*(a+i/10000*(b-a) for a, b in zip(q0, q1))) for i in range(10001)]
    print(f'max_deviation={max(map(deviation,dense)):.4f}')
    print(f'joint_length={sum(math.dist(a,b) for a,b in zip(dense,dense[1:])):.4f}')
    print(f'line_length={math.dist(P0,P1):.4f}')
    bad = path_ik(line_points((.35,0),(-.35,0),10))
    write_csv('unreachable.csv', 'x,y,th1_rad,th2_rad,status', bad)
    assert sum(r[4]=='UNREACHABLE' for r in bad) == 9
    try:
        joint_commands(bad)
    except ValueError:
        print('unreachable=9/11; commands=BLOCKED')
    else:
        raise AssertionError('unsafe command generation')
    return rows


def arcs():
    a0, a1 = math.radians(30), math.radians(120)
    cx, cy = L1*math.cos(a0), L1*math.sin(a0)
    arc = arc_points(cx, cy, L2, a0, a1, 6)
    sweep = [fk2(a0, math.radians(15*i)) for i in range(7)]
    error = max(math.dist(a,b) for a,b in zip(arc,sweep))
    assert error < 1e-12
    write_csv('arc.csv', 'x,y', arc)
    write_csv('sweep.csv', 'x,y', sweep)
    print(f'arc_length={L2*(a1-a0):.4f}; chord={math.dist(arc[0],arc[-1]):.4f}; match_tol=1e-12')
    return arc


def trapezoid():
    rows = path_ik_trap(P0, P1, 2., .5, .1)
    write_csv('path.csv', 't,s,x,y,th1,th2,dth1,dth2,w', rows)
    print('sample_speed_max=' + ','.join(f'{max(abs(r[j]) for r in rows[1:]):.3f}' for j in (6,7)))
    print(f'w_range={min(r[8] for r in rows):.4f},{max(r[8] for r in rows):.4f}')
    return rows


def polynomials():
    q0, q1 = ik2(*P0), ik2(*P1)
    c1, c2 = (joint_cubic(q0[j],q1[j],2.,20) for j in (0,1))
    rows = [(a[0], *map(math.degrees,a[1:]), *map(math.degrees,b[1:])) for a,b in zip(c1,c2)]
    write_csv('cubic.csv', 't,th1,dth1,ddth1,th2,dth2,ddth2', rows)
    dev = [(a[0], *fk2(a[1],b[1]), deviation(fk2(a[1],b[1]))) for a,b in zip(c1,c2)]
    write_csv('dev.csv', 't,x,y,dev', dev)
    assert abs(dev[10][3]-.2004845) < 1e-6
    print(f'cubic_mid_deviation={dev[10][3]:.4f}')
    print(f'cubic_peak_speed={math.degrees(q1[0]-q0[0])*1.5/2:.3f}')
    print(f'quintic_peak_speed={math.degrees(q1[0]-q0[0])*1.875/2:.3f}')
    return rows


if __name__ == '__main__':
    compare()
    arcs()
    trapezoid()
    polynomials()
