"""EP.14 역기구학·자코비안 끝내기 — 다섯 답안 묶음을 한 모듈로.

지문의 신호어에서 계산을 고르는 solve() 디스패처와, 다섯 묶음이 쓰는
계산 함수를 모두 여기에 둔다. 각도 단위는 함수 이름·인자 주석에 명시한다.

  ① 해석 IK   ik2 / reach_check      (#222 #242 #465 #477)
  ② 작업공간   reach_manip / hull / shoelace / point_in_polygon (#257 #497 #287)
  ③ 속도·힘   jac / velocity / torque / gravity_torque          (#239 #260)
  ④ 수치 IK   newton_dls / jpinv_ik3 / null_dir                 (#253)
  ⑤ 안정성    com_projection / support_margin                   (#484)

출력 규약(KIN-63·KIN-64): 내부 계산은 반올림하지 않고, 출력만 소수 4자리.
수렴 종료는 ‖e‖ < 1e-6 m. 반복 횟수는 '오차 검사 횟수'와 '각도 갱신 횟수'를
구분해 함께 돌려준다.
"""
import math

TOL = 1e-6
G = 9.81


# ---------------------------------------------------------------- 공통 도구
def wrap(a):
    """각도(rad)를 -pi ~ pi 대표 범위로 되감는다 (#477)."""
    return math.atan2(math.sin(a), math.cos(a))


def fk2(a1, a2, t1, t2):
    """2링크 순기구학. 각도는 rad."""
    return (a1 * math.cos(t1) + a2 * math.cos(t1 + t2),
            a1 * math.sin(t1) + a2 * math.sin(t1 + t2))


def fk_n(lens, angs):
    """n링크 순기구학 — 누적각으로 가로·세로 성분을 더한다."""
    x = y = c = 0.0
    for l, t in zip(lens, angs):
        c += t
        x += l * math.cos(c)
        y += l * math.sin(c)
    return (x, y)


def jac(a1, a2, t1, t2):
    """2링크 자코비안 — 열 = 관절, 행 = 손끝속도 성분."""
    s1, c1 = math.sin(t1), math.cos(t1)
    s12, c12 = math.sin(t1 + t2), math.cos(t1 + t2)
    return [[-a1 * s1 - a2 * s12, -a2 * s12],
            [a1 * c1 + a2 * c12, a2 * c12]]


def jac_n(lens, angs):
    """n링크 자코비안 (2 x n). 열 j = 관절 j만 돌렸을 때의 손끝 변화율."""
    cum, pts = 0.0, []
    for l, t in zip(lens, angs):
        cum += t
        pts.append((cum, l))
    J = [[0.0] * len(lens), [0.0] * len(lens)]
    for j in range(len(lens)):
        for k in range(j, len(lens)):
            c, l = pts[k]
            J[0][j] += -l * math.sin(c)
            J[1][j] += l * math.cos(c)
    return J


def det2(J):
    return J[0][0] * J[1][1] - J[0][1] * J[1][0]


def svd2(J):
    """2x2 특이값 (sigma_max, sigma_min) — J^T J 고윳값의 제곱근."""
    a = J[0][0] ** 2 + J[1][0] ** 2
    b = J[0][0] * J[0][1] + J[1][0] * J[1][1]
    d = J[0][1] ** 2 + J[1][1] ** 2
    tr, det = a + d, a * d - b * b
    disc = max(0.0, tr * tr / 4 - det)
    hi, lo = tr / 2 + math.sqrt(disc), tr / 2 - math.sqrt(disc)
    return (math.sqrt(max(0.0, hi)), math.sqrt(max(0.0, lo)))


# ------------------------------------------------------------- ① 해석 IK
def ik2(a1, a2, x, y, elbow='down'):
    """해석적 역기구학. 반환 각도는 rad. elbow-down 은 theta2 >= 0 (#222 규약)."""
    r2 = x * x + y * y
    r = math.sqrt(r2)
    if not (abs(a1 - a2) - 1e-12 <= r <= a1 + a2 + 1e-12):
        return None
    c2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2)
    c2 = max(-1.0, min(1.0, c2))                 # 부동소수 미세 오차만 클램핑
    s2 = math.sqrt(max(0.0, 1 - c2 * c2))
    if elbow == 'up':
        s2 = -s2
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(y, x) - math.atan2(a2 * s2, a1 + a2 * c2)
    return (wrap(t1), wrap(t2))


def reach_check(a1, a2, x, y):
    """도달 범위 판정 — 닿지 않으면 예외가 아니라 문자열을 돌려준다."""
    r = math.hypot(x, y)
    r2 = r * r
    c2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2)
    if abs(a1 - a2) - 1e-12 <= r <= a1 + a2 + 1e-12:
        return {'status': 'REACHABLE', 'r': r, 'c2': c2}
    return {'status': 'UNREACHABLE', 'r': r, 'c2': c2}


def trilaterate(beacons, ranges):
    """삼변측량 (#465) — 원의 방정식 차로 만든 2x2 선형계를 크레이머로 푼다."""
    (x0, y0), (x1, y1), (x2, y2) = beacons
    r0, r1, r2 = ranges
    A = [[2 * (x1 - x0), 2 * (y1 - y0)],
         [2 * (x2 - x0), 2 * (y2 - y0)]]
    b = [r0 * r0 - r1 * r1 - x0 * x0 + x1 * x1 - y0 * y0 + y1 * y1,
         r0 * r0 - r2 * r2 - x0 * x0 + x2 * x2 - y0 * y0 + y2 * y2]
    D = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    if abs(D) < 1e-12:
        return None
    Dx = b[0] * A[1][1] - A[0][1] * b[1]
    Dy = A[0][0] * b[1] - b[0] * A[1][0]
    return (Dx / D, Dy / D, D)


def circle_intersections(c0, r0, c1, r1):
    """두 원의 교점 — 2링크 IK 두 해의 팔꿈치 위치."""
    (x0, y0), (x1, y1) = c0, c1
    dx, dy = x1 - x0, y1 - y0
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r0 * r0 - a * a))
    xm, ym = x0 + a * dx / d, y0 + a * dy / d
    return [(xm + h * dy / d, ym - h * dx / d),
            (xm - h * dy / d, ym + h * dx / d)]


# ---------------------------------------------------------- ② 작업공간
def reach_manip(a1, a2, x, y):
    """#257 — 도달 판정 + 조작성 지수 w + 조건수 kappa."""
    chk = reach_check(a1, a2, x, y)
    if chk['status'] == 'UNREACHABLE':
        return {'status': 'UNREACHABLE', 'r': round(chk['r'], 4)}
    t1, t2 = ik2(a1, a2, x, y)
    J = jac(a1, a2, t1, t2)
    smax, smin = svd2(J)
    return {'status': 'REACHABLE', 'r': round(chk['r'], 4),
            'w': round(abs(det2(J)), 4),
            'sigma_max': round(smax, 4), 'sigma_min': round(smin, 4),
            'kappa': round(smax / smin, 4) if smin > 0 else float('inf')}


def annulus_area(a1, a2):
    """환형 작업공간 넓이 — 바깥 원에서 안쪽 원을 뺀다."""
    return math.pi * ((a1 + a2) ** 2 - (a1 - a2) ** 2)


def cross(o, a, b):
    """방향 판정 — 양수면 왼쪽으로 꺾임, 0이면 일직선."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def hull(points):
    """#497 모노톤 체인 볼록껍질 — 반시계 순서, 시작점 중복 없음."""
    P = sorted(set(points))
    if len(P) < 3:
        return P
    ch = []
    for path in (P, P[::-1]):
        start = len(ch) + 2
        for p in path:
            while len(ch) >= start and cross(ch[-2], ch[-1], p) <= 0:
                ch.pop()
            ch.append(p)
        ch.pop()
    return ch


def shoelace(P):
    """#287 신발끈 공식 — 마지막 다음은 첫 꼭짓점."""
    n = len(P)
    return abs(sum(P[i][0] * P[(i + 1) % n][1] - P[(i + 1) % n][0] * P[i][1]
                   for i in range(n))) / 2


def point_in_polygon(pt, poly):
    """가로 반직선 교차 수로 내부·외부를 가린다."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def sweep_points(a1, a2, t1_deg, t2_deg, n1, n2):
    """관절각 격자를 훑어 얻은 손끝 점열 (양 끝 포함)."""
    out = []
    for i in range(n1):
        t1 = math.radians(t1_deg[0] + (t1_deg[1] - t1_deg[0]) * i / (n1 - 1))
        for j in range(n2):
            t2 = math.radians(t2_deg[0] + (t2_deg[1] - t2_deg[0]) * j / (n2 - 1))
            out.append(fk2(a1, a2, t1, t2))
    return out


def occupancy_area(points, cell=0.01):
    """격자 점유 면적 추정 — 볼록껍질 넓이와 구별한다."""
    cells = {(math.floor(x / cell), math.floor(y / cell)) for x, y in points}
    return len(cells) * cell * cell, len(cells)


def cspace_grid(a1, a2, t1_deg, t2_deg, n, obstacle):
    """구성공간 격자 — 팔꿈치와 손끝 두 점만 원 장애물과 검사한다."""
    (ox, oy), orad = obstacle
    free = blocked = 0
    for i in range(n):
        t1 = math.radians(t1_deg[0] + (t1_deg[1] - t1_deg[0]) * i / (n - 1))
        for j in range(n):
            t2 = math.radians(t2_deg[0] + (t2_deg[1] - t2_deg[0]) * j / (n - 1))
            ex, ey = a1 * math.cos(t1), a1 * math.sin(t1)
            tx, ty = fk2(a1, a2, t1, t2)
            hit = (math.hypot(ex - ox, ey - oy) <= orad or
                   math.hypot(tx - ox, ty - oy) <= orad)
            if hit:
                blocked += 1
            else:
                free += 1
    return free, blocked


# ------------------------------------------------------------ ③ 속도·힘
def velocity(J, dq):
    """#239  v = J qdot  (그냥 곱한다)."""
    return (J[0][0] * dq[0] + J[0][1] * dq[1],
            J[1][0] * dq[0] + J[1][1] * dq[1])


def torque(J, F):
    """#260  tau = J^T F  (행과 열을 바꾼다)."""
    return (J[0][0] * F[0] + J[1][0] * F[1],
            J[0][1] * F[0] + J[1][1] * F[1])


def central_diff(a1, a2, q, dq, h=1e-5):
    """중앙차분 검산 — 앞뒤 위치 차를 전체 시간 차로 나눈다."""
    p = fk2(a1, a2, q[0] + h * dq[0], q[1] + h * dq[1])
    m = fk2(a1, a2, q[0] - h * dq[0], q[1] - h * dq[1])
    return ((p[0] - m[0]) / (2 * h), (p[1] - m[1]) / (2 * h))


def inv_jac_qdot(J, v):
    """qdot = J^-1 v — 분모가 det J 라 특이점 근처에서 커진다."""
    d = det2(J)
    if d == 0:
        return None
    return ((J[1][1] * v[0] - J[0][1] * v[1]) / d,
            (-J[1][0] * v[0] + J[0][0] * v[1]) / d)


def gravity_torque(a1, a2, m1, m2, t1, t2, g=G):
    """수직면 2링크의 중력 보상 토크 — 무게중심은 각 링크 중앙."""
    tg2 = (m2 * a2 / 2) * g * math.cos(t1 + t2)
    tg1 = (m1 * a1 / 2 + m2 * a1) * g * math.cos(t1) + tg2
    return (tg1, tg2)


# ------------------------------------------------------------ ④ 수치 IK
def newton_dls(goal, th0, a1, a2, lam=0.0, tol=TOL, itmax=200):
    """뉴턴-랩슨(lam=0) 또는 감쇠 최소자승(lam>0) 반복.

    반환 dict 의 checks = 오차 검사 횟수, updates = 각도 갱신 횟수.
    th_raw 는 되감기 전, th_wrapped 는 되감은 각도다.
    """
    t1, t2 = th0
    errs = []
    for it in range(1, itmax + 1):
        px, py = fk2(a1, a2, t1, t2)
        ex, ey = goal[0] - px, goal[1] - py
        e = math.hypot(ex, ey)
        errs.append(e)
        if e < tol:
            return {'converged': True, 'checks': it, 'updates': it - 1,
                    'th_raw': (t1, t2), 'th_wrapped': (wrap(t1), wrap(t2)),
                    'errs': errs, 'fk': (px, py)}
        J = jac(a1, a2, t1, t2)
        if lam > 0:                                  # (J^T J + lam^2 I)^-1 J^T e
            A = J[0][0] ** 2 + J[1][0] ** 2 + lam * lam
            B = J[0][0] * J[0][1] + J[1][0] * J[1][1]
            C = J[0][1] ** 2 + J[1][1] ** 2 + lam * lam
            g0 = J[0][0] * ex + J[1][0] * ey
            g1 = J[0][1] * ex + J[1][1] * ey
            D = A * C - B * B
            if D == 0:
                return {'converged': False, 'checks': it, 'updates': it - 1,
                        'reason': 'singular', 'errs': errs}
            t1 += (C * g0 - B * g1) / D
            t2 += (-B * g0 + A * g1) / D
        else:
            step = inv_jac_qdot(J, (ex, ey))
            if step is None:
                return {'converged': False, 'checks': it, 'updates': it - 1,
                        'reason': 'det J = 0', 'errs': errs}
            t1 += step[0]
            t2 += step[1]
    return {'converged': False, 'checks': itmax, 'updates': itmax - 1,
            'reason': 'itmax', 'errs': errs}


def _mat2_inv(M):
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]], d


def jpinv(J):
    """의사역행렬 J+ = J^T (J J^T)^-1 — 각 반복의 보정량이 최소노름해."""
    n = len(J[0])
    JJt = [[sum(J[i][k] * J[j][k] for k in range(n)) for j in range(2)] for i in range(2)]
    inv, d = _mat2_inv(JJt)
    Jp = [[sum(J[k][i] * inv[k][j] for k in range(2)) for j in range(2)] for i in range(n)]
    return Jp, JJt, d


def jpinv_ik3(goal, th0, lens, tol=TOL, itmax=200):
    """3링크(2 x 3 자코비안) 수치 IK — 매 반복의 보정량을 최소노름으로 고른다."""
    th = list(th0)
    errs = []
    for it in range(1, itmax + 1):
        px, py = fk_n(lens, th)
        ex, ey = goal[0] - px, goal[1] - py
        e = math.hypot(ex, ey)
        errs.append(e)
        if e < tol:
            return {'converged': True, 'checks': it, 'updates': it - 1,
                    'th': tuple(th), 'fk': (px, py), 'errs': errs}
        J = jac_n(lens, th)
        Jp, _, _ = jpinv(J)
        for i in range(len(th)):
            th[i] += Jp[i][0] * ex + Jp[i][1] * ey
    return {'converged': False, 'checks': itmax, 'updates': itmax - 1, 'errs': errs}


def null_dir(J):
    """2 x 3 자코비안의 널공간 방향 — 세 2x2 소행렬식의 교대합."""
    a, b, c = J[0]
    d, e, f = J[1]
    n = (b * f - c * e, c * d - a * f, a * e - b * d)
    norm = math.sqrt(sum(v * v for v in n))
    return tuple(v / norm for v in n)


# ----------------------------------------------------------- ⑤ 안정성
def com_projection(a1, a2, t1, t2, m_base, m1, m2, m_load, base_com=(0.0, 0.0)):
    """질량 가중 평균으로 무게중심을 구해 수평 지면에 투영한다."""
    e = (a1 * math.cos(t1), a1 * math.sin(t1))
    tip = fk2(a1, a2, t1, t2)
    parts = [(m_base, base_com),
             (m1, (e[0] / 2, e[1] / 2)),
             (m2, ((e[0] + tip[0]) / 2, (e[1] + tip[1]) / 2)),
             (m_load, tip)]
    total = sum(m for m, _ in parts)
    cx = sum(m * p[0] for m, p in parts) / total
    cy = sum(m * p[1] for m, p in parts) / total
    return (cx, cy), total, tip


def support_margin(point, half=0.30):
    """정사각 지지면에서 경계까지 남은 거리 — 음수면 지지면 밖이다."""
    return min(half - abs(point[0]), half - abs(point[1]))


def polygon_margin(point, poly):
    """일반 볼록 다각형 — 변마다 부호 있는 수직거리의 최솟값."""
    best = None
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        L = math.hypot(ex, ey)
        signed = ((point[0] - x1) * ey - (point[1] - y1) * ex) / L
        best = -signed if best is None else min(best, -signed)
    return best


# ------------------------------------------------------- 판별 디스패처
def solve(case):
    """지문의 주어진 값과 요구 산출물에서 계산 경로를 고른다."""
    g, want = case['given'], case['want']
    if 'F' in g:
        return statics_case(g)                 # N · N·m  -> tau = J^T F
    if 'dq' in g:
        return velocity_case(g)                # rad/s    -> v = J qdot
    if want == 'reach':
        return reach_manip(g['a1'], g['a2'], g['x'], g['y'])
    if want == 'angles' and 'th0' in g:
        return newton_dls((g['x'], g['y']), g['th0'], g['a1'], g['a2'],
                          lam=g.get('lam', 0.0))
    if want == 'angles':
        return {e: ik2(g['a1'], g['a2'], g['x'], g['y'], e) for e in ('down', 'up')}
    if want == 'area':
        return shoelace(hull(g['pts']))
    if want == 'stability':
        com, total, _ = com_projection(g['a1'], g['a2'], g['th'][0], g['th'][1],
                                       g['m_base'], g['m1'], g['m2'], g['m_load'])
        return {'com': com, 'total': total, 'margin': support_margin(com, g['half'])}
    raise ValueError('판별 실패: 주어진 값과 요구 산출물을 다시 읽을 것')


def velocity_case(g):
    J = jac(g['a1'], g['a2'], g['th'][0], g['th'][1])
    v = velocity(J, g['dq'])
    return {'J': J, 'det': det2(J), 'v': v, 'speed': math.hypot(*v),
            'check': central_diff(g['a1'], g['a2'], g['th'], g['dq'])}


def statics_case(g):
    J = jac(g['a1'], g['a2'], g['th'][0], g['th'][1])
    tau = torque(J, g['F'])
    wrong = velocity(J, g['F'])
    return {'J': J, 'tau': tau, 'no_transpose': wrong,
            'gap': math.hypot(tau[0] - wrong[0], tau[1] - wrong[1])}
