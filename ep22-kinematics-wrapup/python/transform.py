import math

def rigid2(p, angle, t):
    c, s = math.cos(angle), math.sin(angle)
    return c*p[0]-s*p[1]+t[0], s*p[0]+c*p[1]+t[1]
