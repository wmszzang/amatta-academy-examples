"""링크 두께와 자체 충돌을 제외한 선분–원 충돌 표본."""
import math
from kin_common import fk

def seg_hit(a, b, centre, radius):
    dx, dy = b[0]-a[0], b[1]-a[1]
    u = max(0.0, min(1.0, ((centre[0]-a[0])*dx+(centre[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(a[0]+u*dx-centre[0], a[1]+u*dy-centre[1]) <= radius

def cspace_grid(a1=2, a2=1.5, obs=(1.6,1.2), radius=.35, step=5):
    if step <= 0 or abs(360/step-round(360/step)) > 1e-9:
        raise ValueError('step must divide 360')
    free, hit = [], []
    for i in range(round(360/step)):
        for j in range(round(360/step)):
            d1, d2 = -180+i*step, -180+j*step
            t1, t2 = math.radians(d1), math.radians(d2)
            elbow = (a1*math.cos(t1), a1*math.sin(t1))
            tip = fk(t1,t2,a1,a2)
            bad = seg_hit((0,0),elbow,obs,radius) or seg_hit(elbow,tip,obs,radius)
            (hit if bad else free).append((d1,d2))
    return free, hit

if __name__ == '__main__':
    free, hit = cspace_grid()
    print(f'obstacle={len(hit)} free={len(free)} total={len(hit)+len(free)}')
