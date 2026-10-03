import math
from ik import ik2
from fk import fk2

def arc_points(center,radius,start,end,n):
    return [(center[0]+radius*math.cos(start+(end-start)*i/n),center[1]+radius*math.sin(start+(end-start)*i/n)) for i in range(n+1)]

def joint_lerp(a,b,u):
    return tuple(x+(y-x)*u for x,y in zip(a,b))

def trapezoid(distance,vmax,amax,t):
    if distance<0 or vmax<=0 or amax<=0: raise ValueError('INVALID_LIMIT')
    ta=min(vmax/amax,math.sqrt(distance/amax)); vp=amax*ta
    cruise=0 if vp==0 else max(0,(distance-amax*ta*ta)/vp)
    total=2*ta+cruise; t=max(0,min(t,total))
    if t<ta: return .5*amax*t*t,amax*t,total
    if t<ta+cruise: return .5*amax*ta*ta+vp*(t-ta),vp,total
    rem=total-t
    return distance-.5*amax*rem*rem,amax*rem,total

def timed_arc(center,radius,start,distance,vmax,amax,t):
    s,v,total=trapezoid(distance,vmax,amax,t)
    a=start+s/radius
    return (center[0]+radius*math.cos(a),center[1]+radius*math.sin(a)),v,total

def cubic_approach(q0,qf,t,total):
    if total<=0: raise ValueError('INVALID_DURATION')
    u=max(0,min(1,t/total))
    return q0+(qf-q0)*(3*u*u-2*u*u*u),(qf-q0)*6*u*(1-u)/total
