import math

def geodesic_deg(q1, q2):
    """단위 쿼터니언의 최소 자세 차이, 도 단위."""
    d = abs(sum(a*b for a,b in zip(q1,q2)))
    return math.degrees(2*math.acos(min(1.0,d)))
