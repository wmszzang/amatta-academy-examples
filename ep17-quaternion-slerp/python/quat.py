import math

def qmul(a, b):
    """성분 순서 (w,x,y,z), 입력 순서 그대로 a 곱하기 b."""
    w,x,y,z = a
    W,X,Y,Z = b
    return (w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
            w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W)

def qnorm(q):
    n = math.sqrt(sum(c*c for c in q))
    if n == 0:
        raise ValueError('영 쿼터니언은 자세를 나타내지 않습니다.')
    return tuple(c/n for c in q)

def conj(q):
    return (q[0], -q[1], -q[2], -q[3])

def axis_deg(axis, degrees):
    half = math.radians(degrees)/2
    length = math.sqrt(sum(x*x for x in axis))
    return (math.cos(half), *(x/length*math.sin(half) for x in axis))

def rotate(q, v):
    q = qnorm(q)
    return qmul(qmul(q, (0, *v)), conj(q))[1:]

def fmt(q):
    # 반올림 직전의 미세한 음수로 -0.0000이 생기지 않게 합니다.
    return ','.join(f'{0.0 if abs(c)<0.00005 else c:.4f}' for c in q)
