import sys,json,math,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from kin import *
import numpy as np

def close(a,b,tol=5e-5):
    assert np.allclose(a,b,rtol=0,atol=tol),(a,b)

with tempfile.TemporaryDirectory() as temp:
    actual=run(temp)
expected=json.loads(Path(__file__).with_name('expected.json').read_text())
for key,value in expected.items(): close(actual[key],value)
assert ik2((3,0),(1,1)) is None
for fn,args in [(project,((1,1,0),600,320,240)),(project,((1,1,-1),600,320,240)),(backproject,(392,210,392,600,.12,320,240)),(backproject,(392,210,400,600,.12,320,240))]:
    try: fn(*args)
    except ValueError: pass
    else: raise AssertionError('invalid camera input accepted')
for pitch in [math.pi/2,-math.pi/2]:
    r=rotation_zyx(-.4,pitch,1.1);q=euler_zyx(r);close(rotation_zyx(*q),r,1e-9)
a,t,r,corrected=estimate_svd(Q,[(-x,y) for x,y in Q]);assert corrected;close(np.linalg.det(r),1,1e-9)
close(mecanum_fk(mecanum_ik(.9,-.4,.25,.05,.3,.26),.05,.3,.26),(.9,-.4,.25),1e-12)
for p in arc_points((1.2,.6),.5,math.pi/6,5*math.pi/6,6): close(fk2(ik2(p,(1.2,.9)),(1.2,.9)),p,1e-12)
for distance in [.1,10.]:
    s,v,T=trapezoid(distance,1.,2.,0);close((s,v),(0,0),1e-12)
    close(trapezoid(distance,1.,2.,T)[:2],(distance,0),1e-12)
close(cubic_approach(.2,1.,0,2),(.2,0));close(cubic_approach(.2,1.,2,2),(1,0))
q=(.3,.6);qd=(.5,-.2);h=1e-6
p0=fk2(q,(2,1.5));p1=fk2(tuple(a+h*b for a,b in zip(q,qd)),(2,1.5))
close(tuple((b-a)/h for a,b in zip(p0,p1)),velocity(q,qd,(2,1.5)),1e-6)
print('PASS: expected values, units/order, unreachable, invalid camera, both gimbal poles, reflection, path, timing, finite difference')
