"""오른손·고정축·열벡터 회전. 쿼터니언 순서는 (w,x,y,z).

>>> [round(x, 4) for x in rodrigues((1,2,2), math.pi/3)[0]]
[0.5556, -0.4662, 0.6885]
>>> R_to_axis_angle(rodrigues((0,1,0), math.pi))[0]
[0.0, 1.0, 0.0]
"""
import argparse
import csv
import math
from pathlib import Path
import sys


def normalize(values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite input')
    norm = math.hypot(*values)
    if norm == 0:
        raise ValueError('zero length input')
    return [v / norm for v in values]


def skew(k):
    x, y, z = k
    return [[0, -z, y], [z, 0, -x], [-y, x, 0]]


def matmul(a, b):
    return [[sum(a[i][t]*b[t][j] for t in range(3))
             for j in range(3)] for i in range(3)]


def rodrigues(k, theta):
    k = normalize(k)
    if not math.isfinite(theta):
        raise ValueError('non-finite angle')
    K = skew(k)
    K2 = matmul(K, K)
    s, c = math.sin(theta), 1-math.cos(theta)
    return [[float(i == j) + s*K[i][j] + c*K2[i][j]
             for j in range(3)] for i in range(3)]


def axis_angle_to_quat(k, theta):
    k = normalize(k)
    if not math.isfinite(theta):
        raise ValueError('non-finite angle')
    return [math.cos(theta/2)] + [v*math.sin(theta/2) for v in k]


def quat_to_R(q):
    w, x, y, z = normalize(q)
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]


def determinant(R):
    return sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]
                        - R[1][(i+2)%3]*R[2][(i+1)%3]) for i in range(3))


def orthogonal_error(R):
    return max(abs(sum(R[t][i]*R[t][j] for t in range(3))-float(i == j))
               for i in range(3) for j in range(3))


def validate_rotation(R):
    if len(R) != 3 or any(len(row) != 3 for row in R):
        raise ValueError('matrix must be 3 by 3')
    if not all(math.isfinite(v) for row in R for v in row):
        raise ValueError('non-finite matrix')
    if orthogonal_error(R) > 1e-8 or abs(determinant(R)-1) > 1e-8:
        raise ValueError('input is not a rotation matrix')


def R_to_quat(R):
    validate_rotation(R)
    t = sum(R[i][i] for i in range(3))
    if t > 0:
        s = 2*math.sqrt(1+t)
        q = [s/4, (R[2][1]-R[1][2])/s,
             (R[0][2]-R[2][0])/s, (R[1][0]-R[0][1])/s]
    else:
        # 동률이면 작은 인덱스를 먼저 골라 분기를 결정적으로 유지한다.
        i = max(range(3), key=lambda j: R[j][j])
        j, h = (i+1)%3, (i+2)%3
        s = 2*math.sqrt(max(0, 1+R[i][i]-R[j][j]-R[h][h]))
        q = [(R[h][j]-R[j][h])/s, 0, 0, 0]
        q[i+1], q[j+1], q[h+1] = s/4, (R[i][j]+R[j][i])/s, (R[i][h]+R[h][i])/s
    q = normalize(q)
    # 영도~반 바퀴 범위를 선택하고 반 바퀴에서는 최대 축 성분을 양수로 둔다.
    if q[0] < 0:
        q = [-v for v in q]
    return q


def R_to_axis_angle(R):
    q = R_to_quat(R)
    cosine = max(-1, min(1, (sum(R[i][i] for i in range(3))-1)/2))
    theta = math.acos(cosine)
    length = math.hypot(*q[1:])
    if length < 1e-12:
        return None, 0.0
    # 반 바퀴 부근에는 작은 sin(theta)로 나누지 않는다.
    if abs(math.sin(theta)) < 1e-6:
        theta = 2*math.atan2(length, q[0])
        return [v/length for v in q[1:]], theta
    return [(R[2][1]-R[1][2])/(2*math.sin(theta)),
            (R[0][2]-R[2][0])/(2*math.sin(theta)),
            (R[1][0]-R[0][1])/(2*math.sin(theta))], theta


def selfcheck():
    count = 0
    for k in [(1,0,0), (0,1,0), (0,0,1), (1,2,2), (-2,1,3)]:
        for theta in [0, math.pi/2, math.pi/3, math.pi, math.pi-1e-8, math.pi+1e-8]:
            R = rodrigues(k, theta)
            q = axis_angle_to_quat(k, theta)
            back = R_to_quat(R)
            assert abs(abs(sum(a*b for a,b in zip(q,back)))-1) < 1e-12
            assert max(abs(a-b) for ra,rb in zip(R,quat_to_R(q)) for a,b in zip(ra,rb)) < 1e-12
            assert quat_to_R(q) == quat_to_R([-v for v in q])
            axis, angle = R_to_axis_angle(R)
            if axis is None:
                assert angle == 0
            else:
                again = rodrigues(axis, angle)
                assert max(abs(a-b) for ra,rb in zip(R,again) for a,b in zip(ra,rb)) < 1e-8
            assert orthogonal_error(R) < 1e-12 and abs(determinant(R)-1) < 1e-12
            count += 1
    for fn, values in [(rodrigues, ([0,0,0], 1)), (quat_to_R, ([0,0,0,0],)),
                       (R_to_quat, ([[1,0,0],[0,1,0],[0,0,-1]],))]:
        try:
            fn(*values)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid input was accepted')
    print('SELF CHECK: 30 round trips + 3 invalid inputs PASS')
    return count


def fmt(v):
    return f'{0.0 if abs(v) < 0.00005 else v:.4f}'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['axis','quat','matrix'], nargs='?', default='axis')
    p.add_argument('--out', default='.')
    p.add_argument('--selfcheck', action='store_true')
    args = p.parse_args()
    if args.selfcheck:
        selfcheck()
        return
    values = [float(v) for v in sys.stdin.read().split()]
    if len(values) != (9 if args.mode == 'matrix' else 4):
        raise ValueError('wrong input count')
    if args.mode == 'axis':
        R = rodrigues(values[:3], values[3])
    elif args.mode == 'quat':
        R = quat_to_R(values)
    else:
        R = [values[i:i+3] for i in range(0,9,3)]
    q = R_to_quat(R)
    axis, angle = R_to_axis_angle(R)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, rows in [('R',R),('quat',[q]),
                       ('axis_angle', [[*(axis or [0,0,0]), angle, int(axis is not None)]])]:
        with (out/(name+'.csv')).open('w', newline='', encoding='utf-8') as f:
            csv.writer(f, lineterminator='\n').writerows([[fmt(v) for v in row] for row in rows])
        print(name + ':')
        for row in rows:
            print(','.join(fmt(v) for v in row))
    print(f'det={determinant(R):.12f}; orthogonal_error={orthogonal_error(R):.3e}')


if __name__ == '__main__':
    try:
        main()
    except ValueError as exc:
        print('INPUT ERROR: '+str(exc), file=sys.stderr)
        sys.exit(2)
