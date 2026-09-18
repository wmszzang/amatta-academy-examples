"""EP.13 정역학 — tau = J^T F · 중력 보상 토크 · 지지 다각형 정적 안정 여유.

길이·속도는 숫자만 쓰고, 힘과 토크에만 뉴턴(N)·뉴턴 미터(N*m)를 붙인다.
"""
import argparse
import csv
import math
from pathlib import Path

from numeric_ik import fk2, jacobian, write_csv

G = 9.81


def torque(J, F):
    """tau = J^T F — 전치는 행이 아니라 열을 읽는다."""
    return [J[0][0] * F[0] + J[1][0] * F[1],
            J[0][1] * F[0] + J[1][1] * F[1]]


def wrong_torque(J, F):
    """전치를 빠뜨린 오답. 채점 루브릭이 그대로 경고하는 실수다."""
    return [J[0][0] * F[0] + J[0][1] * F[1],
            J[1][0] * F[0] + J[1][1] * F[1]]


def gravity_torque(t1, t2, a1=2.0, a2=1.5, m1=3.0, m2=2.0, g=G):
    """무게중심이 링크 중앙일 때의 닫힌형 중력 보상 토크."""
    lc1, lc2 = a1 / 2, a2 / 2
    tau2 = m2 * lc2 * g * math.cos(t1 + t2)
    tau1 = (m1 * lc1 + m2 * a1) * g * math.cos(t1) + tau2
    return [tau1, tau2]


def gravity_torque_jacobian(t1, t2, a1=2.0, a2=1.5, m1=3.0, m2=2.0, g=G):
    """링크별 무게중심 자코비안 합산: sum Jc_i^T (0, -m_i g). 닫힌형과 교차 검산한다."""
    lc1, lc2 = a1 / 2, a2 / 2
    s1, c1 = math.sin(t1), math.cos(t1)
    s12, c12 = math.sin(t1 + t2), math.cos(t1 + t2)
    Jc1 = [[-lc1 * s1, 0.0], [lc1 * c1, 0.0]]
    Jc2 = [[-a1 * s1 - lc2 * s12, -lc2 * s12], [a1 * c1 + lc2 * c12, lc2 * c12]]
    t_1 = torque(Jc1, (0.0, -m1 * g))
    t_2 = torque(Jc2, (0.0, -m2 * g))
    return [-(t_1[0] + t_2[0]), -(t_1[1] + t_2[1])]


def support_margin(polygon, point):
    """반시계 규약의 지지 다각형에서 부호거리 최솟값을 구한다.

    안쪽이면 모든 변에서 양수다. 한 변에서라도 음수면 밖, 불안정이다.
    """
    best = None
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        cross = ex * (point[1] - ay) - ey * (point[0] - ax)
        d = cross / math.hypot(ex, ey)
        best = d if best is None else min(best, d)
    return (best >= 0.0), best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='output-python')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    a1, a2 = 2.0, 1.5
    t1 = t2 = 0.5

    print('== #260 자코비안 전치 (a 2,1.5 / theta 0.5,0.5 / F 3,-1 N) ==')
    J = jacobian(a1, a2, t1, t2)
    print('  J 1행  ', ' '.join(f'{v:.4f}' for v in J[0]))
    print('  J 2행  ', ' '.join(f'{v:.4f}' for v in J[1]))
    print(f'  det J   {J[0][0]*J[1][1]-J[0][1]*J[1][0]:.4f}  (= a1 a2 sin(theta2))')
    F = (3.0, -1.0)
    tau = torque(J, F)
    bad = wrong_torque(J, F)
    print(f'  tau = J^T F  ({tau[0]:.4f}, {tau[1]:.4f}) N*m')
    print(f'  전치 누락    ({bad[0]:.4f}, {bad[1]:.4f})  <- 오답')

    print('== 가상일 검증 (qdot 0.3,-0.7) ==')
    qd = (0.3, -0.7)
    xd = [J[0][0] * qd[0] + J[0][1] * qd[1], J[1][0] * qd[0] + J[1][1] * qd[1]]
    lhs = F[0] * xd[0] + F[1] * xd[1]
    rhs = tau[0] * qd[0] + tau[1] * qd[1]
    print(f'  xdot ({xd[0]:.4f}, {xd[1]:.4f})   F.xdot {lhs:.10f}   tau.qdot {rhs:.10f}')

    print('== 중력 보상 토크 (m 3,2 kg / lc 링크 중앙 / g 9.81) ==')
    poses = [('기본 0.5, 0.5', 0.5, 0.5), ('수평 0, 0', 0.0, 0.0), ('수직 pi/2, 0', math.pi / 2, 0.0)]
    rows = []
    for name, p1, p2 in poses:
        tg = gravity_torque(p1, p2)
        tj = gravity_torque_jacobian(p1, p2)
        print(f'  {name:<14} 닫힌형 ({tg[0]:.4f}, {tg[1]:.4f})   자코비안 합산 ({tj[0]:.4f}, {tj[1]:.4f}) N*m')
        rows.append([f'{p1:.6f}', f'{p2:.6f}', f'{tg[0]:.6f}', f'{tg[1]:.6f}',
                     f'{tj[0]:.6f}', f'{tj[1]:.6f}'])
    pay = torque(J, (0.0, 2.0 * G))
    tg = gravity_torque(t1, t2)
    print(f'  2 kg 페이로드 유지 ({pay[0]:.4f}, {pay[1]:.4f}) N*m')
    print(f'  합계             ({tg[0]+pay[0]:.4f}, {tg[1]+pay[1]:.4f}) N*m')
    write_csv(out / 'statics.csv',
              ['theta1', 'theta2', 'taug1_closed', 'taug2_closed', 'taug1_jac', 'taug2_jac'], rows)

    print('== #484 지지 다각형 정적 안정 여유 ==')
    square = [(0, 0), (1, 0), (1, 1), (0, 1)]
    tri = [(0, 0), (2, 0), (1, 1.5)]
    cases = [(square, (0.5, 0.5)), (square, (0.9, 0.5)), (square, (1.2, 0.5)), (tri, (1.0, 0.4))]
    lines = []
    for poly, p in cases:
        ok, margin = support_margin(poly, p)
        tag = '정사각형' if poly is square else '삼각형'
        line = f'  {tag} P({p[0]}, {p[1]})  ->  {"안정" if ok else "불안정"}, 여유 {margin:.4f}'
        print(line)
        lines.append(f'{tag},{p[0]},{p[1]},{ok},{margin:.6f}')
    (out / 'stability.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    write_csv(out / 'torque.csv', ['case', 'tau1', 'tau2'],
              [['J^T F', f'{tau[0]:.6f}', f'{tau[1]:.6f}'],
               ['J F (오답)', f'{bad[0]:.6f}', f'{bad[1]:.6f}'],
               ['payload 2kg', f'{pay[0]:.6f}', f'{pay[1]:.6f}']])
    print(f'출력 폴더: {out.resolve()}')


if __name__ == '__main__':
    main()
