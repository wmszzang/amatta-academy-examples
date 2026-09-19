"""다섯 지문을 순서대로 풀고 채점용 CSV·콘솔 출력을 만든다.

실행: python run_all.py [출력폴더]
모든 출력 수치는 소수 4자리, 각도 열은 이름에 단위(_deg/_rad)를 적는다.
반복 횟수 열 iters 는 '오차 검사 횟수'이며 각도 갱신 횟수는 그보다 1 적다.
"""
import csv
import math
import os
import sys

from kin14 import (annulus_area, central_diff, circle_intersections,
                   com_projection, cspace_grid, det2, fk2, fk_n, gravity_torque,
                   hull, inv_jac_qdot, ik2, jac, jac_n, jpinv, jpinv_ik3,
                   newton_dls, null_dir, occupancy_area, point_in_polygon,
                   reach_check, reach_manip, shoelace, support_margin, svd2,
                   sweep_points, torque, trilaterate, velocity, wrap)

OUT = sys.argv[1] if len(sys.argv) > 1 else 'output-python'
FENCE = [(-1.8, -0.4), (3.8, -0.4), (3.8, 2.4), (1.0, 3.8), (-1.8, 2.4)]


def w4(v):
    return round(v, 4)


def main():
    os.makedirs(OUT, exist_ok=True)
    print('=== (1) 해석 IK ===')
    rows = []

    # #222 — 도 단위, elbow-down
    t1, t2 = ik2(1.0, 1.0, 1.5, 1.0, 'down')
    fk = fk2(1.0, 1.0, t1, t2)
    r222 = math.hypot(1.5, 1.0)
    c2_222 = (r222 ** 2 - 2) / 2
    print('#222 r=%.4f cos(th2)=%.4f th=(%.4f, %.4f) deg  FK=(%.4f, %.4f)'
          % (r222, c2_222, math.degrees(t1), math.degrees(t2), fk[0], fk[1]))
    print('     전체 정밀도 th1=%.6f deg  th2=%.6f deg'
          % (math.degrees(t1), math.degrees(t2)))
    rows.append(['222', 'down', w4(math.degrees(t1)), w4(math.degrees(t2)),
                 w4(t1), w4(t2), w4(fk[0]), w4(fk[1]), 'REACHABLE'])

    # #242 — 라디안, 두 분기
    for elbow in ('down', 'up'):
        a1, a2 = ik2(2.0, 1.5, 2.5, 1.5, elbow)
        f = fk2(2.0, 1.5, a1, a2)
        print('#242 %-4s th=(%.4f, %.4f) rad  = (%.4f, %.4f) deg  FK=(%.4f, %.4f)'
              % (elbow, a1, a2, math.degrees(a1), math.degrees(a2), f[0], f[1]))
        rows.append(['242', elbow, w4(math.degrees(a1)), w4(math.degrees(a2)),
                     w4(a1), w4(a2), w4(f[0]), w4(f[1]), 'REACHABLE'])

    # 도달 불가 두 사례
    for (x, y) in ((3.0, 2.0), (0.3, 0.2)):
        c = reach_check(2.0, 1.5, x, y)
        print('도달 판정 (%.1f, %.1f) r=%.4f cos(th2)=%.4f -> %s'
              % (x, y, c['r'], c['c2'], c['status']))
        rows.append(['242', 'target(%.1f,%.1f)' % (x, y), '', '', '', '',
                     '', '', c['status']])

    with open(os.path.join(OUT, 'p1_ik.csv'), 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(['case', 'elbow', 'th1_deg', 'th2_deg', 'th1_rad', 'th2_rad',
                    'fk_x', 'fk_y', 'status'])
        w.writerows(rows)

    # #465 삼변측량
    sol = trilaterate([(0, 0), (4, 0), (0, 4)],
                      [math.sqrt(2), math.sqrt(10), math.sqrt(10)])
    print('#465 삼변측량 det A=%.4f -> (%.4f, %.4f)' % (sol[2], sol[0], sol[1]))
    elbows = circle_intersections((0.0, 0.0), 2.0, (2.5, 1.5), 1.5)
    print('     2링크 원 교점(팔꿈치) = (%.4f, %.4f) / (%.4f, %.4f)'
          % (elbows[0][0], elbows[0][1], elbows[1][0], elbows[1][1]))

    # #477 각도 정규화
    print('#477 3.0 - (-3.0) = %.4f -> wrap %.4f' % (6.0, wrap(3.0 - (-3.0))))
    print('     0.1 - (-0.1) = %.4f -> wrap %.4f' % (0.2, wrap(0.2)))
    up1 = ik2(2.0, 1.5, 2.5, 1.5, 'up')[0]
    print('     elbow-up th1 + 2pi = %.4f -> wrap %.4f' % (up1 + 2 * math.pi, wrap(up1 + 2 * math.pi)))
    down = ik2(2.0, 1.5, 2.5, 1.5, 'down')
    up = ik2(2.0, 1.5, 2.5, 1.5, 'up')
    print('     전환 비용 |wrap(d th1)|=%.4f rad (%.4f deg), |wrap(d th2)|=%.4f rad'
          % (abs(wrap(up[0] - down[0])), math.degrees(abs(wrap(up[0] - down[0]))),
             abs(wrap(up[1] - down[1]))))

    print()
    print('=== (2) 작업공간 ===')
    rm = reach_manip(2.0, 1.5, 2.5, 1.5)
    print('#257', rm)
    print('     환형 넓이 = %.4f m^2' % annulus_area(2.0, 1.5))
    print('     th2 스윕 (w, sigma_max, sigma_min, kappa)')
    sweep_rows = []
    for deg in (5, 30, math.degrees(math.acos(0.375)), 90, 120, 175):
        t = math.radians(deg)
        J = jac(2.0, 1.5, 0.0, t)
        smax, smin = svd2(J)
        print('      %8.4f deg  w=%.4f  smax=%.4f  smin=%.4f  kappa=%.4f'
              % (deg, abs(det2(J)), smax, smin, smax / smin))
        sweep_rows.append([w4(deg), w4(abs(det2(J))), w4(smax), w4(smin), w4(smax / smin)])

    pts = sweep_points(2.0, 1.5, (0, 90), (0, 120), 25, 25)
    H = hull(pts)
    area_h = shoelace(H)
    # 점유 면적은 촘촘한 표본(각 관절 901개)을 0.01 m 격자에 넣어 추정한다.
    dense = sweep_points(2.0, 1.5, (0, 90), (0, 120), 901, 901)
    area_g, cells = occupancy_area(dense, 0.01)
    outside = sum(1 for p in pts if not point_in_polygon(p, FENCE))
    print('#497 스윕 %d점 -> 껍질 %d정점' % (len(pts), len(H)))
    print('#287 원문 오각형 넓이 = %.4f' % shoelace([(0, 0), (4, 0), (4, 3), (2, 5), (0, 3)]))
    print('     껍질 넓이 = %.4f / 격자 추정 면적 = %.4f (%d칸) -> 껍질이 %.1f%% 크다'
          % (area_h, area_g, cells, (area_h / area_g - 1) * 100))
    print('     펜스 넓이 = %.4f, 펜스 밖 도달점 = %d/%d (%.2f%%)'
          % (shoelace(FENCE), outside, len(pts), 100 * outside / len(pts)))
    free, blocked = cspace_grid(2.0, 1.5, (0, 90), (0, 120), 90, ((2.2, 1.0), 0.5))
    print('     C-space 90x90: C-free=%d / C-obstacle=%d (%.1f%% 차단)'
          % (free, blocked, 100 * blocked / (free + blocked)))

    with open(os.path.join(OUT, 'p2_workspace.csv'), 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(['th1_deg', 'th2_deg', 'x', 'y', 'in_fence'])
        for i in range(25):
            for j in range(25):
                d1, d2 = 90 * i / 24, 120 * j / 24
                x, y = fk2(2.0, 1.5, math.radians(d1), math.radians(d2))
                w.writerow([w4(d1), w4(d2), w4(x), w4(y),
                            int(point_in_polygon((x, y), FENCE))])
    with open(os.path.join(OUT, 'hull.csv'), 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(['x', 'y'])
        w.writerows([[w4(p[0]), w4(p[1])] for p in H])
    with open(os.path.join(OUT, 'area.txt'), 'w', encoding='utf-8') as fp:
        fp.write('hull_area_m2=%.4f\ngrid_area_m2=%.4f\ngrid_cells=%d\n'
                 'fence_area_m2=%.4f\noutside=%d/%d\n'
                 % (area_h, area_g, cells, shoelace(FENCE), outside, len(pts)))

    print()
    print('=== (3) 속도·힘 ===')
    q = (0.5, 0.5)
    J = jac(2.0, 1.5, *q)
    print('J = [[%.4f, %.4f], [%.4f, %.4f]]  det J = %.4f'
          % (J[0][0], J[0][1], J[1][0], J[1][1], det2(J)))
    v = velocity(J, (0.4, -0.3))
    chk = central_diff(2.0, 1.5, q, (0.4, -0.3))
    print('#239 v = (%.4f, %.4f) m/s  |v| = %.4f  중앙차분 = (%.4f, %.4f)'
          % (v[0], v[1], math.hypot(*v), chk[0], chk[1]))
    F = (3.0, -1.0)
    tau = torque(J, F)
    wrong = velocity(J, F)
    print('#260 tau = J^T F = (%.4f, %.4f) N.m' % tau)
    print('     전치 누락 J F = (%.4f, %.4f), 차이 크기 = %.4f'
          % (wrong[0], wrong[1], math.hypot(tau[0] - wrong[0], tau[1] - wrong[1])))
    power_f = F[0] * v[0] + F[1] * v[1]
    power_t = tau[0] * 0.4 + tau[1] * (-0.3)
    print('     가상일 검산 F.v = %.6f W,  tau.qdot = %.6f W' % (power_f, power_t))

    print('     1/det 폭주 (v = (0, 0.2) m/s, th1 = 0.5 고정)')
    burst = []
    for t2v in (0.5, 0.2, 0.05, 0.01, 0.001):
        Jt = jac(2.0, 1.5, 0.5, t2v)
        dq = inv_jac_qdot(Jt, (0.0, 0.2))
        print('      th2=%.3f  det J=%.6f  |qdot|=%.4f rad/s'
              % (t2v, det2(Jt), math.hypot(*dq)))
        burst.append([t2v, round(det2(Jt), 6), w4(math.hypot(*dq))])

    tg = gravity_torque(2.0, 1.5, 3.0, 2.0, *q)
    print('중력 보상 tau_g = (%.4f, %.4f) N.m' % tg)
    print('지령 토크 tau_g + J^T F = (%.4f, %.4f) N.m' % (tg[0] + tau[0], tg[1] + tau[1]))

    with open(os.path.join(OUT, 'p3_vel_force.csv'), 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(['th1_rad', 'th2_rad', 'dq1', 'dq2', 'vx', 'vy', 'Fx', 'Fy',
                    'tau1', 'tau2', 'detJ', 'w', 'kappa'])
        smax, smin = svd2(J)
        w.writerow([w4(q[0]), w4(q[1]), 0.4, -0.3, w4(v[0]), w4(v[1]), 3.0, -1.0,
                    w4(tau[0]), w4(tau[1]), w4(det2(J)), w4(abs(det2(J))),
                    w4(smax / smin)])
        for row in burst:
            w.writerow(['0.5', row[0], '', '', 0.0, 0.2, '', '', '', '', row[1], '', ''])

    print()
    print('=== (4) 수치 IK ===')
    num_rows = []
    for th0, lam in (((0.5, 0.5), 0.0), ((0.0, 0.0), 0.0), ((0.0, 0.0), 0.05),
                     ((0.1, 0.05), 0.0), ((0.1, 0.05), 0.05)):
        res = newton_dls((2.5, 1.5), th0, 2.0, 1.5, lam=lam)
        tag = 'lam=%.2f 시작 (%.2f, %.2f)' % (lam, th0[0], th0[1])
        if res['converged']:
            raw, wr = res['th_raw'], res['th_wrapped']
            print('%-26s 검사 %2d회 / 갱신 %2d회  raw=(%.4f, %.4f) wrap=(%.4f, %.4f)'
                  % (tag, res['checks'], res['updates'], raw[0], raw[1], wr[0], wr[1]))
            num_rows.append([w4(th0[0]), w4(th0[1]), lam, res['checks'], res['updates'],
                             w4(raw[0]), w4(raw[1]), w4(wr[0]), w4(wr[1]), 1])
        else:
            print('%-26s 검사 %2d회 / 갱신 %2d회  실패: %s'
                  % (tag, res['checks'], res['updates'], res['reason']))
            num_rows.append([w4(th0[0]), w4(th0[1]), lam, res['checks'], res['updates'],
                             '', '', '', '', 0])

    base = newton_dls((2.5, 1.5), (0.5, 0.5), 2.0, 1.5)
    print('뉴턴 오차 수열:', ' '.join('%.6g' % e for e in base['errs']))
    dls0 = newton_dls((2.5, 1.5), (0.0, 0.0), 2.0, 1.5, lam=0.05)
    print('DLS 오차 수열 :', ' '.join('%.4f' % e for e in dls0['errs']))

    lens = (2.0, 1.5, 1.0)
    th0 = (0.3, 0.4, 0.2)
    J3 = jac_n(lens, th0)
    Jp, JJt, d = jpinv(J3)
    print('3링크 J = [[%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f]]'
          % (J3[0][0], J3[0][1], J3[0][2], J3[1][0], J3[1][1], J3[1][2]))
    print('      J J^T = [[%.4f, %.4f], [%.4f, %.4f]]  det = %.4f'
          % (JJt[0][0], JJt[0][1], JJt[1][0], JJt[1][1], d))
    r3 = jpinv_ik3((2.6, 1.6), th0, lens)
    print('      검사 %d회 / 갱신 %d회 -> th = (%.4f, %.4f, %.4f)  FK = (%.4f, %.4f)'
          % (r3['checks'], r3['updates'], r3['th'][0], r3['th'][1], r3['th'][2],
             r3['fk'][0], r3['fk'][1]))
    print('      전체 관절 이동량 합 = %.4f rad'
          % sum(abs(r3['th'][i] - th0[i]) for i in range(3)))
    n = null_dir(jac_n(lens, r3['th']))
    Jn = jac_n(lens, r3['th'])
    prod = (sum(Jn[0][i] * n[i] for i in range(3)), sum(Jn[1][i] * n[i] for i in range(3)))
    moved = tuple(r3['th'][i] + 0.15 * n[i] for i in range(3))
    tip = fk_n(lens, moved)
    print('      널 방향 n = (%.4f, %.4f, %.4f)  J n = (%.2e, %.2e)'
          % (n[0], n[1], n[2], prod[0], prod[1]))
    print('      0.15 rad 이동 -> 끝점 (%.6f, %.6f), 목표 이탈 %.6f m'
          % (tip[0], tip[1], math.hypot(tip[0] - 2.6, tip[1] - 1.6)))

    with open(os.path.join(OUT, 'p4_numeric.csv'), 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(['start_th1_rad', 'start_th2_rad', 'lam', 'iters', 'updates',
                    'th1_raw_rad', 'th2_raw_rad', 'th1_wrapped_rad',
                    'th2_wrapped_rad', 'converged'])
        w.writerows(num_rows)

    print()
    print('=== (5) 안정성 ===')
    stab_rows = []
    for name, th in (('folded', (0.5, 0.5)), ('extended', (0.1, 0.05))):
        com, total, tip = com_projection(2.0, 1.5, th[0], th[1], 30.0, 3.0, 2.0, 1.0)
        m = support_margin(com, 0.30)
        Js = jac(2.0, 1.5, *th)
        smax, smin = svd2(Js)
        print('#484 %-9s 총질량 %.1f kg  말단 (%.4f, %.4f)  CoM 투영 (%.4f, %.4f)  여유 %+.4f m  %s'
              % (name, total, tip[0], tip[1], com[0], com[1], m,
                 '안정' if m >= 0 else '전도'))
        print('       그 자세의 w=%.4f  kappa=%.4f' % (abs(det2(Js)), smax / smin))
        stab_rows.append([name, w4(com[0]), w4(com[1]), w4(m), int(m >= 0)])

    with open(os.path.join(OUT, 'p5_stability.csv'), 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(['pose', 'com_x', 'com_y', 'margin', 'stable'])
        w.writerows(stab_rows)

    print()
    print('출력 폴더:', os.path.abspath(OUT))


if __name__ == '__main__':
    main()
