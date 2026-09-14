"""EP.11: 관절속도, 말단속도와 특이점의 수치 검산."""
import csv
import math
from pathlib import Path


def jacobian(a1, a2, t1, t2):
    s1, c1 = math.sin(t1), math.cos(t1)
    s12, c12 = math.sin(t1 + t2), math.cos(t1 + t2)
    return [[-a1*s1 - a2*s12, -a2*s12],
            [a1*c1 + a2*c12, a2*c12]]


def ee_vel(j, d1, d2):
    return (j[0][0]*d1 + j[0][1]*d2, j[1][0]*d1 + j[1][1]*d2)


def det2(j):
    return j[0][0]*j[1][1] - j[0][1]*j[1][0]


def is_singular(j, eps=1e-6):
    # 작은 분모를 피하는 예제용 수치 기준이며 물리적 안전 기준이 아니다.
    return abs(det2(j)) < eps


def joint_vel(j, vx, vy, eps=1e-6):
    d = det2(j)
    if abs(d) < eps:
        return None
    return ((j[1][1]*vx - j[0][1]*vy)/d,
            (-j[1][0]*vx + j[0][0]*vy)/d)


def position(a1, a2, t1, t2):
    return (a1*math.cos(t1) + a2*math.cos(t1+t2),
            a1*math.sin(t1) + a2*math.sin(t1+t2))


def planar_jacobian(lengths, angles):
    directions = [sum(angles[:i+1]) for i in range(len(angles))]
    return [[sum(-lengths[k]*math.sin(directions[k]) for k in range(i, len(angles))) for i in range(len(angles))],
            [sum(lengths[k]*math.cos(directions[k]) for k in range(i, len(angles))) for i in range(len(angles))]]


def wrist_det(beta):
    return abs(math.sin(beta))


def fd_check(a1=2.0, a2=1.5, t1=0.5236, t2=0.7854, h=1e-6):
    cols = []
    for i in range(2):
        plus, minus = [t1, t2], [t1, t2]
        plus[i] += h
        minus[i] -= h
        p, m = position(a1, a2, *plus), position(a1, a2, *minus)
        cols.append([(p[k]-m[k])/(2*h) for k in range(2)])
    return [[cols[c][r] for c in range(2)] for r in range(2)]


def sweep_row(t2):
    t1 = math.pi/6
    j = jacobian(2.0, 1.5, t1, t2)
    p = position(2.0, 1.5, t1, t2)
    r = math.hypot(*p)
    q = joint_vel(j, 0.1*p[0]/r, 0.1*p[1]/r)
    return (t2, det2(j), *q, math.hypot(*q))


def joint2_limit_angle():
    lo, hi = 0.001, 0.1
    for _ in range(70):
        mid = (lo+hi)/2
        if abs(sweep_row(mid)[3]) > math.pi:
            lo = mid
        else:
            hi = mid
    return (lo+hi)/2


def write_outputs(directory='.'):
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    j = jacobian(2.0, 1.5, 0.5236, 0.7854)
    xd, yd = ee_vel(j, 0.1, 0.2)
    (out/'vel.txt').write_text(f'{xd:.4f} {yd:.4f}\n', encoding='utf-8')
    def csv_out(name, header, rows):
        with (out/name).open('w', newline='', encoding='utf-8') as f:
            w = csv.writer(f, lineterminator='\n')
            w.writerow(header)
            w.writerows([[f'{v:.10f}' for v in row] for row in rows])
    csv_out('jac.csv', ['j1', 'j2'], j)
    csv_out('sweep.csv', ['theta2', 'det', 'qd1', 'qd2', 'qnorm'],
            [sweep_row(t) for t in (0.7854, 0.4, 0.2, 0.1, 0.05, 0.02, 0.01, 0.001)])
    csv_out('wrist.csv', ['beta', 'abs_det'], [(b, wrist_det(b)) for b in (0.7854, 0.2, 0.0)])
    (out/'singular.txt').write_text(
        'outer: rank=1 r=3.5000 lost=30deg available=120deg\n'
        'inner-boundary-2link: rank=1 r=0.5000\n'
        'internal-3link: rank=1 r=0.5000 workspace=[0,4.5]\n', encoding='utf-8')
    print(f'{xd:.4f} {yd:.4f}')
    print(f'det={det2(j):.4f} speed={math.hypot(xd, yd):.4f}')
    print(f'joint2 limit: theta2={joint2_limit_angle():.7f} rad')
    return j


if __name__ == '__main__':
    write_outputs()
