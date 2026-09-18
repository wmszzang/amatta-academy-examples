"""EP.13 수치 역기구학 — 뉴턴-랩슨 · DLS · 의사역행렬 · 널공간.

외부 라이브러리를 쓰지 않는다. 표준 math 모듈 하나로 전부 계산한다.
각도는 내부에서 라디안, CSV 출력은 라디안과 도를 함께 적는다.
"""
import argparse
import csv
import math
from pathlib import Path

TOL = 1e-6
ITMAX = 200
SING = 1e-12


def fk2(l1, l2, t1, t2):
    """관절 각도 -> 손끝 좌표(순기구학)."""
    return (l1 * math.cos(t1) + l2 * math.cos(t1 + t2),
            l1 * math.sin(t1) + l2 * math.sin(t1 + t2))


def jacobian(l1, l2, t1, t2):
    s1, c1 = math.sin(t1), math.cos(t1)
    s12, c12 = math.sin(t1 + t2), math.cos(t1 + t2)
    return [[-l1 * s1 - l2 * s12, -l2 * s12],
            [l1 * c1 + l2 * c12, l2 * c12]]


def det2(J):
    return J[0][0] * J[1][1] - J[0][1] * J[1][0]


def newton_step(goal, th, l1, l2, step=1.0):
    """dtheta = step * J^-1 e. 특이 자세면 (None, err)로 DLS 전환을 알린다."""
    t1, t2 = th
    px, py = fk2(l1, l2, t1, t2)
    ex, ey = goal[0] - px, goal[1] - py
    J = jacobian(l1, l2, t1, t2)
    d = det2(J)
    if abs(d) < SING:
        return None, math.hypot(ex, ey)
    t1 += step * (J[1][1] * ex - J[0][1] * ey) / d
    t2 += step * (-J[1][0] * ex + J[0][0] * ey) / d
    return (t1, t2), math.hypot(ex, ey)


def dls_step(goal, th, l1, l2, lam=0.1):
    """dtheta = J^T (J J^T + lam^2 I)^-1 e. lam > 0이면 나눗셈이 죽지 않는다."""
    t1, t2 = th
    px, py = fk2(l1, l2, t1, t2)
    ex, ey = goal[0] - px, goal[1] - py
    J = jacobian(l1, l2, t1, t2)
    a11 = J[0][0] ** 2 + J[0][1] ** 2 + lam * lam
    a12 = J[0][0] * J[1][0] + J[0][1] * J[1][1]
    a22 = J[1][0] ** 2 + J[1][1] ** 2 + lam * lam
    det = a11 * a22 - a12 * a12
    u = (a22 * ex - a12 * ey) / det
    v = (-a12 * ex + a11 * ey) / det
    return (t1 + J[0][0] * u + J[1][0] * v,
            t2 + J[0][1] * u + J[1][1] * v), math.hypot(ex, ey)


def wrap(a):
    """관절각을 [-pi, pi]로 접는다. 래핑하지 않으면 -2505도 같은 값이 그대로 남는다."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def solve(goal, init, l1=2.0, l2=1.5, step=1.0, lam=None, tol=TOL, itmax=ITMAX):
    """반복 횟수 = 관절각을 실제로 갱신한 횟수. 오차 검사 횟수는 그보다 1 많다."""
    th = tuple(init)
    errs = []
    status = 'max_iter'
    for _ in range(itmax):
        if lam is None:
            nxt, err = newton_step(goal, th, l1, l2, step)
        else:
            nxt, err = dls_step(goal, th, l1, l2, lam)
        errs.append(err)
        if err < tol:
            status = 'converged'
            break
        if nxt is None:
            status = 'singular'
            break
        th = nxt
    else:
        errs.append(math.hypot(goal[0] - fk2(l1, l2, *th)[0], goal[1] - fk2(l1, l2, *th)[1]))
    return {'theta': th, 'wrapped': (wrap(th[0]), wrap(th[1])),
            'errors': errs, 'updates': len(errs) - (1 if status == 'converged' else 0),
            'checks': len(errs), 'status': status,
            'fk': fk2(l1, l2, *th), 'max_err': max(errs)}


def jacobian3(a, th):
    """3링크 평면 자코비안(2행 3열). 첫 열은 항상 (-y, x)와 같다."""
    cum = [th[0], th[0] + th[1], th[0] + th[1] + th[2]]
    row0, row1 = [], []
    for k in range(3):
        row0.append(-sum(a[i] * math.sin(cum[i]) for i in range(k, 3)))
        row1.append(sum(a[i] * math.cos(cum[i]) for i in range(k, 3)))
    return [row0, row1]


def fk3(a, th):
    cum = [th[0], th[0] + th[1], th[0] + th[1] + th[2]]
    return (sum(a[i] * math.cos(cum[i]) for i in range(3)),
            sum(a[i] * math.sin(cum[i]) for i in range(3)))


def pinv_vel(J, xd):
    """qdot = J^+ xdot = J^T (J J^T)^-1 xdot — 최소노름해."""
    b11 = sum(J[0][k] ** 2 for k in range(3))
    b12 = sum(J[0][k] * J[1][k] for k in range(3))
    b22 = sum(J[1][k] ** 2 for k in range(3))
    det = b11 * b22 - b12 * b12
    u = (b22 * xd[0] - b12 * xd[1]) / det
    v = (-b12 * xd[0] + b11 * xd[1]) / det
    return [J[0][k] * u + J[1][k] * v for k in range(3)]


def null_dir(J):
    """2x3의 널공간 방향 = 두 행의 외적(정규화)."""
    r0, r1 = J[0], J[1]
    n = [r0[1] * r1[2] - r0[2] * r1[1],
         r0[2] * r1[0] - r0[0] * r1[2],
         r0[0] * r1[1] - r0[1] * r1[0]]
    s = math.sqrt(sum(x * x for x in n))
    return [x / s for x in n]


def mul(J, q):
    return [sum(J[r][k] * q[k] for k in range(len(q))) for r in range(2)]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def write_csv(path, header, rows):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='output-python')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    l1, l2 = 2.0, 1.5

    print('== #253 뉴턴-랩슨 (goal 2.5,1.5 / init 0.5,0.5 / step 1.0) ==')
    r = solve((2.5, 1.5), (0.5, 0.5), l1, l2)
    print('  오차열 ', ' '.join(f'{e:.4g}' for e in r['errors']))
    print(f"  수렴각  ({r['theta'][0]:.4f}, {r['theta'][1]:.4f})  갱신 {r['updates']}회 / 오차 검사 {r['checks']}회")
    print(f"  FK 검산 ({r['fk'][0]:.4f}, {r['fk'][1]:.4f})   상태 {r['status']}")
    write_csv(out / 'newton.csv', ['it', 't1', 't2', 'err'],
              [[i, f'{r["theta"][0]:.10f}', f'{r["theta"][1]:.10f}', f'{e:.12g}']
               for i, e in enumerate(r['errors'])])

    print('== 초기값 의존 (init 2.0,-1.0) ==')
    d = solve((2.5, 1.5), (2.0, -1.0), l1, l2)
    print(f"  엘보 다운 해 ({d['theta'][0]:.4f}, {d['theta'][1]:.4f})  갱신 {d['updates']}회")

    print('== step 감쇠 스윕 ==')
    step_rows = []
    for s in (1.0, 0.7, 0.5, 0.2):
        rs = solve((2.5, 1.5), (0.5, 0.5), l1, l2, step=s)
        step_rows.append([s, rs['updates'], f'{rs["max_err"]:.6g}'])
        print(f'  step {s:<4} -> {rs["updates"]:>3}회')
    write_csv(out / 'step_sweep.csv', ['step', 'updates', 'max_err'], step_rows)

    print('== 특이점 출발 무감쇠 (goal 3.4,0.15 / init 0,0) ==')
    dead = solve((3.4, 0.15), (0.0, 0.0), l1, l2)
    print(f'  상태 {dead["status"]} — 첫 나눗셈에서 중단(det J = 0)')
    print('== 0.001 비틀어 출발한 폭주 ==')
    blow = solve((3.4, 0.15), (0.0, 0.001), l1, l2)
    print('  오차열 앞 3개 ', ' '.join(f'{e:.4g}' for e in blow['errors'][:3]))
    print(f'  최대 오차 {blow["max_err"]:.4f}   착지각 ({blow["theta"][0]:.4f}, {blow["theta"][1]:.4f}) rad'
          f' = ({math.degrees(blow["theta"][0]):.1f}, {math.degrees(blow["theta"][1]):.1f})도')
    print(f'  FK 검산 ({blow["fk"][0]:.4f}, {blow["fk"][1]:.4f})  래핑 후 ({blow["wrapped"][0]:.4f}, {blow["wrapped"][1]:.4f})')

    print('== DLS (같은 목표·같은 특이 초기값, lam 0.1) ==')
    dls = solve((3.4, 0.15), (0.0, 0.0), l1, l2, lam=0.1)
    print('  오차열 ', ' '.join(f'{e:.4g}' for e in dls['errors']))
    print(f"  수렴각  ({dls['theta'][0]:.4f}, {dls['theta'][1]:.4f})  갱신 {dls['updates']}회")
    print(f"  FK 검산 ({dls['fk'][0]:.4f}, {dls['fk'][1]:.4f})")
    write_csv(out / 'dls.csv', ['it', 't1', 't2', 'err'],
              [[i, f'{dls["theta"][0]:.10f}', f'{dls["theta"][1]:.10f}', f'{e:.12g}']
               for i, e in enumerate(dls['errors'])])

    print('== lambda 스윕 ==')
    lam_rows = []
    for lam in (0.01, 0.05, 0.1, 0.3, 1.0):
        rl = solve((3.4, 0.15), (0.0, 0.0), l1, l2, lam=lam)
        lam_rows.append([lam, rl['updates'], f'{rl["max_err"]:.6g}'])
        print(f'  lam {lam:<5} -> {rl["updates"]:>3}회   중간 최대 오차 {rl["max_err"]:.4f}')
    write_csv(out / 'lam_sweep.csv', ['lam', 'updates', 'max_err'], lam_rows)

    print('== 3링크 의사역행렬·널공간 (a 2,1.5,1.0 / theta 0.5,0.5,0.5) ==')
    a3, th3 = (2.0, 1.5, 1.0), (0.5, 0.5, 0.5)
    J3 = jacobian3(a3, th3)
    end = fk3(a3, th3)
    print('  J 1행 ', ' '.join(f'{v:.4f}' for v in J3[0]))
    print('  J 2행 ', ' '.join(f'{v:.4f}' for v in J3[1]))
    print(f'  검산: 첫 열 = (-y, x) = ({-end[1]:.4f}, {end[0]:.4f})')
    qd = pinv_vel(J3, (0.0, 1.0))
    n = null_dir(J3)
    print('  qdot  ', ' '.join(f'{v:.4f}' for v in qd), f'   노름 {norm(qd):.4f}')
    print('  J qdot', ' '.join(f'{v:.4f}' for v in mul(J3, qd)))
    print('  n     ', ' '.join(f'{v:.4f}' for v in n), '   J n', ' '.join(f'{v:.4f}' for v in mul(J3, n)))
    pin_rows = []
    for alpha in (0.0, 0.5, 1.0):
        q = [qd[k] + alpha * n[k] for k in range(3)]
        xd = mul(J3, q)
        pin_rows.append([alpha] + [f'{v:.10f}' for v in q] + [f'{norm(q):.10f}'] +
                        [f'{v:.10f}' for v in xd])
        print(f'  alpha {alpha:<4} 노름 {norm(q):.4f}   손끝 속도 ({xd[0]:.4f}, {xd[1]:.4f})')
    write_csv(out / 'pinv.csv', ['alpha', 'qd1', 'qd2', 'qd3', 'norm', 'xdot', 'ydot'], pin_rows)
    print(f'출력 폴더: {out.resolve()}')


if __name__ == '__main__':
    main()
