import math

def jacobian(q,lengths):
    a,b=q; l1,l2=lengths
    return ((-l1*math.sin(a)-l2*math.sin(a+b),-l2*math.sin(a+b)),
            (l1*math.cos(a)+l2*math.cos(a+b),l2*math.cos(a+b)))

def velocity(q,qd,lengths):
    return tuple(sum(x*y for x,y in zip(row,qd)) for row in jacobian(q,lengths))

def manipulability(q,lengths):
    return abs(lengths[0]*lengths[1]*math.sin(q[1]))
