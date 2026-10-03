import math

def ik2(p, lengths, elbow=1):
    x,y=p; a,b=lengths
    c=(x*x+y*y-a*a-b*b)/(2*a*b)
    if abs(c)>1+1e-12: return None
    c=max(-1,min(1,c)); s=elbow*math.sqrt(max(0,1-c*c))
    return math.atan2(y,x)-math.atan2(b*s,a+b*c), math.atan2(s,c)

def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi
