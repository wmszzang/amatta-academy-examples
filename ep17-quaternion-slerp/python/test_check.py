import math
from quat import axis_deg,qmul,qnorm,conj,rotate
from slerp import slerp,nlerp
from distance import geodesic_deg

def close(a,b,tol=1e-8):
    assert len(a)==len(b)
    assert max(abs(x-y) for x,y in zip(a,b))<tol,(a,b)

I=(1,0,0,0)
Z=axis_deg((0,0,1),90)
X=axis_deg((1,0,0),90)
a=(.7071,0,0,.7071)
close(qmul(I,(0,1,0,0)),(0,1,0,0))
close(qmul(a,a),(0,0,0,.99998082))
close(qnorm(qmul(a,a)),(0,0,0,1))
close((math.sqrt(sum(c*c for c in a)),),(.9999904099540154,))
close(qmul(Z,X),(.5,.5,.5,.5))
close(qmul(X,Z),(.5,.5,-.5,.5))
close((geodesic_deg(qmul(Z,X),qmul(X,Z)),),(120,))
close(rotate(qmul(X,Z),(1,0,0)),(0,0,1))
close(rotate(qmul(Z,X),(1,0,0)),(0,1,0))
close(qmul(Z,conj(Z)),I)
for t,angle in ((.25,22.5),(.5,45),(.75,67.5)):
    close((geodesic_deg(I,slerp(I,Z,t)),),(angle,))
    close(slerp(I,tuple(-c for c in Z),t),slerp(I,Z,t))
close(slerp(Z,Z,.4),Z)
E=axis_deg((0,0,1),170)
sa=[geodesic_deg(I,slerp(I,E,i/10)) for i in range(11)]
la=[geodesic_deg(I,nlerp(I,E,i/10)) for i in range(11)]
close([sa[i+1]-sa[i] for i in range(10)],[17]*10)
close([la[i+1]-la[i] for i in range(10)],
      [12.5123,14.8838,17.3448,19.4888,20.7703,20.7703,19.4888,17.3448,14.8838,12.5123],1e-4)
close((geodesic_deg(I,nlerp(I,E,.25)),),(35.7688,),1e-4)
th=math.acos(-Z[0])
long=tuple(math.sin(th/2)/math.sin(th)*(a-b) for a,b in zip(I,Z))
close((math.degrees(2*math.atan2(long[3],long[0])),),(-135,))
close((math.degrees(2*math.acos(.9995)),),(3.6238542761,))
close([geodesic_deg(I,Z),geodesic_deg(I,(-1,0,0,0)),geodesic_deg(I,(0,0,0,1)),geodesic_deg(Z,X)],[90,0,180,120])
close((math.degrees(2*math.acos(-1)),),(360,))
try:
    math.acos(1.0000000002)
    raise AssertionError('범위 밖 입력이 통과했습니다.')
except ValueError:
    pass
assert math.acos(min(1,1.0000000002))==0
close((geodesic_deg(axis_deg((0,0,1),88),Z),),(2,))
close(qmul(conj(axis_deg((0,0,1),88)),Z),(.9998476952,0,0,.0174524064))
close((.5*.5-(-.5*.5),),(.5,))
euler=qmul(axis_deg((0,0,1),45),axis_deg((1,0,0),45))
mid=slerp(Z,X,.5)
close((geodesic_deg(Z,euler),geodesic_deg(Z,mid),geodesic_deg(euler,mid)),(62.7994,60,19.4712),1e-4)
try:
    qnorm((0,0,0,0))
    raise AssertionError('영 벡터 입력이 통과했습니다.')
except ValueError:
    pass
print('PASS: Hamilton, normalization, order, interpolation branches, distances and counterexamples')
