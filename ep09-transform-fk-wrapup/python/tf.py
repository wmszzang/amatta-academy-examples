"""EP.9 공구함 모듈 tf.py — 2D 강체변환 · 동차변환 체인 · 다링크 FK · DH · 스윕 · 각도 도구 · SAT.

강의 S33·S34가 약속한 함수군을 한 파일에 둔다. 각도 인자는 모두 도(degree)이며
함수 안에서만 라디안으로 바꾼다. 외부 라이브러리 없이 math만 쓴다.
"""
import math

# ---------- 각도 도구 ----------

def wrap(deg):
    """각도를 [-180, 180) 범위로 접는다. wrap(-325) = 35."""
    return (deg + 180.0) % 360.0 - 180.0


def circular_mean(degs):
    """원형 평균 = atan2(sum sin, sum cos). 10·350·20도 → 약 6.7도(단순 평균 126.6667은 오답)."""
    s = sum(math.sin(math.radians(d)) for d in degs)
    c = sum(math.cos(math.radians(d)) for d in degs)
    return math.degrees(math.atan2(s, c))


# ---------- 2D 강체변환 ----------

def rot2(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s], [s, c]]


def rigid2d(points, deg, t):
    """p' = R p + t (회전 후 이동). 리스트 컴프리헨션 한 줄."""
    (a, b), (c, d) = rot2(deg)
    return [(a * x + b * y + t[0], c * x + d * y + t[1]) for x, y in points]


def inv2d(deg, t):
    """역변환의 (각도, 이동항). 이동항은 -R^T t 이지 단순 -t 가 아니다."""
    (a, b), (c, d) = rot2(deg)
    return -deg, (-(a * t[0] + c * t[1]), -(b * t[0] + d * t[1]))


def recover_angle(p_orig, p_new, t):
    """변환된 한 점에서 회전각 복원: (atan2 각, 내적·acos 각). 변환의 역이지 역기구학이 아니다."""
    vx, vy = p_new[0] - t[0], p_new[1] - t[1]
    by_atan2 = math.degrees(math.atan2(vy, vx)) - math.degrees(math.atan2(p_orig[1], p_orig[0]))
    dot = p_orig[0] * vx + p_orig[1] * vy
    norm = math.hypot(*p_orig) * math.hypot(vx, vy)
    by_acos = math.degrees(math.acos(max(-1.0, min(1.0, dot / norm))))
    return by_atan2, by_acos


# ---------- 4x4 동차변환 ----------

def rotz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rotx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def homog(R, t):
    """[R t; 0 1] 4x4 조립."""
    return [[R[0][0], R[0][1], R[0][2], t[0]],
            [R[1][0], R[1][1], R[1][2], t[1]],
            [R[2][0], R[2][1], R[2][2], t[2]],
            [0.0, 0.0, 0.0, 1.0]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def apply(T, p):
    return tuple(T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3] for i in range(3))


def inv4(T):
    """[R^T  -R^T t; 0 1]."""
    Rt = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-(Rt[i][0] * T[0][3] + Rt[i][1] * T[1][3] + Rt[i][2] * T[2][3]) for i in range(3)]
    return homog(Rt, t)


# ---------- 다링크 순기구학 ----------

def fk(links, angles_deg):
    """상대 관절각을 누적해 원점부터 끝점까지 관절 좌표 목록(튜플)을 반환."""
    x = y = cum = 0.0
    pts = [(0.0, 0.0)]
    for l, a in zip(links, angles_deg):
        cum += a
        x += l * math.cos(math.radians(cum))
        y += l * math.sin(math.radians(cum))
        pts.append((x, y))
    return pts


def sweep(links, theta1_deg, start, stop, step):
    """theta1 고정, theta2 를 start..stop(양 끝 포함) 로 훑는 FK 반복."""
    return [(a, fk(links, (theta1_deg, a))[-1]) for a in range(start, stop + 1, step)]


# ---------- DH ----------

def dh(a, alpha_deg, d, theta_deg):
    """표준 DH 행렬 A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ca, sa = math.cos(math.radians(alpha_deg)), math.sin(math.radians(alpha_deg))
    ct, st = math.cos(math.radians(theta_deg)), math.sin(math.radians(theta_deg))
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def dh_chain(rows):
    """[(a, alpha, d, theta), ...] 을 차례로 곱한 T."""
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for row in rows:
        T = mul(T, dh(*row))
    return T


def yaw_of(T):
    """회전 블록 첫 열(끝 링크 x축)에서 자세각 atan2(T21, T11)."""
    return math.degrees(math.atan2(T[1][0], T[0][0]))


# ---------- 분리축 정리(SAT) ----------

def obb_corners(box):
    """box = (cx, cy, hw, hh, deg). 로컬(±hw, ±hh)를 회전 후 중심에 더한다."""
    cx, cy, hw, hh, deg = box
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(cx + sx * c - sy * s, cy + sx * s + sy * c)
            for sx, sy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))]


def obb_axes(box):
    c, s = math.cos(math.radians(box[4])), math.sin(math.radians(box[4]))
    return [(c, s), (-s, c)]


def project(corners, axis):
    vals = [x * axis[0] + y * axis[1] for x, y in corners]
    return min(vals), max(vals)


def sat_obb(box1, box2):
    """네 축(두 상자의 로컬 x·y) 정사영 구간 비교. 반환: (충돌 여부, 축별 (구간1, 구간2, 겹침) 목록)."""
    c1, c2 = obb_corners(box1), obb_corners(box2)
    report = []
    hit = True
    for axis in obb_axes(box1) + obb_axes(box2):
        i1, i2 = project(c1, axis), project(c2, axis)
        overlap = not (i1[1] < i2[0] or i2[1] < i1[0])
        report.append((i1, i2, overlap))
        hit = hit and overlap
    return hit, report
