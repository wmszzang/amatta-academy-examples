"""회전 표현 끝내기: 열벡터·능동 회전, 쿼터니언 (w,x,y,z)."""
import math


def unit(values):
    length = math.sqrt(sum(v * v for v in values))
    if length == 0:
        raise ValueError("길이가 0인 입력은 정규화할 수 없습니다.")
    return [v / length for v in values]


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, value))


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3))
             for j in range(3)] for i in range(3)]


def euler_zyx_to_R(roll, pitch, yaw):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def euler_zyx(R, eps=1e-6):
    sy = math.hypot(R[0][0], R[1][0])
    pitch = math.atan2(-R[2][0], sy)
    if sy < eps:
        # 두 각을 따로 식별할 수 없어 지문이 허용한 대표해를 선택한다.
        return math.atan2(-R[1][2], R[1][1]), pitch, 0.0, True
    return math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0]), False


def rodrigues(k, theta):
    x, y, z = unit(k)
    K = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    K2 = mul(K, K)
    s, c = math.sin(theta), 1-math.cos(theta)
    return [[float(i == j) + s*K[i][j] + c*K2[i][j]
             for j in range(3)] for i in range(3)]


def q_to_R(q):
    w, x, y, z = unit(q)
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]


def R_to_q(R):
    trace = sum(R[i][i] for i in range(3))
    if trace > 0:
        s = 2*math.sqrt(1+trace)
        q = [s/4, (R[2][1]-R[1][2])/s,
             (R[0][2]-R[2][0])/s, (R[1][0]-R[0][1])/s]
    else:
        # 대각합이 작은 반바퀴에서도 작은 수로 나누지 않는다.
        i = max(range(3), key=lambda j: R[j][j])
        j, k = (i+1) % 3, (i+2) % 3
        s = 2*math.sqrt(max(0, 1+R[i][i]-R[j][j]-R[k][k]))
        q = [0.0]*4
        q[0], q[i+1] = (R[k][j]-R[j][k])/s, s/4
        q[j+1], q[k+1] = (R[j][i]+R[i][j])/s, (R[k][i]+R[i][k])/s
    return unit(q)


def hamilton(a, b):
    # #490은 대수적 곱 그대로, #290의 정규화는 호출자가 명시한다.
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def slerp(q0, q1, t):
    q0, q1 = unit(q0), unit(q1)
    d = sum(a*b for a, b in zip(q0, q1))
    if d < 0:
        q1, d = [-v for v in q1], -d
    d = clamp(d)
    if d > 0.9995:
        return unit([a+t*(b-a) for a, b in zip(q0, q1)])
    theta = math.acos(d)
    return [math.sin((1-t)*theta)/math.sin(theta)*a +
            math.sin(t*theta)/math.sin(theta)*b for a, b in zip(q0, q1)]


def geodesic_deg(q0, q1):
    q0, q1 = unit(q0), unit(q1)
    d = abs(sum(a*b for a, b in zip(q0, q1)))
    return math.degrees(2*math.acos(clamp(d, 0, 1)))


def gyro_heading(omega, dt, bias, theta0=0.0):
    if dt <= 0 or not omega:
        raise ValueError("양의 표본 간격과 하나 이상의 표본이 필요합니다.")
    corrected = [w-bias for w in omega]
    return theta0 + sum((a+b)*dt/2 for a, b in zip(corrected, corrected[1:]))
