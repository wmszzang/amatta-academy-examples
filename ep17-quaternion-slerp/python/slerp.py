import math
from quat import qnorm

def slerp(q0, q1, t):
    """단위 입력의 최단 호 보간. 진행률은 0 이상 1 이하입니다."""
    d = sum(a*b for a,b in zip(q0,q1))
    if d < 0:
        q1 = tuple(-c for c in q1)
        d = -d
    if d > 0.9995:
        return qnorm(tuple(a+t*(b-a) for a,b in zip(q0,q1)))
    th = math.acos(max(-1.0,min(1.0,d)))
    s = math.sin(th)
    c0,c1 = math.sin((1-t)*th)/s, math.sin(t*th)/s
    return tuple(c0*a+c1*b for a,b in zip(q0,q1))

def nlerp(q0, q1, t):
    return qnorm(tuple((1-t)*a+t*b for a,b in zip(q0,q1)))
