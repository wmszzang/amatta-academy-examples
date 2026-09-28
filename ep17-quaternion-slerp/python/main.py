import csv
from quat import axis_deg,qmul,qnorm,fmt
from slerp import slerp,nlerp
from distance import geodesic_deg

def results():
    a = (.7071,0,0,.7071)
    identity = (1,0,0,0)
    z = axis_deg((0,0,1),90)
    x = axis_deg((1,0,0),90)
    print('290,'+fmt(qnorm(qmul(a,a))))
    print('490,'+fmt(qmul(a,a)))
    print('identity,'+fmt(qmul(identity,(0,1,0,0))))
    print('z*x,'+fmt(qmul(z,x)))
    print('x*z,'+fmt(qmul(x,z)))
    for t in (.25,.5,.75):
        print(f'259:{t:.2f},'+fmt(slerp(identity,z,t)))
    for target in (z,(-1,0,0,0),(0,0,0,1)):
        print('559,'+fmt((geodesic_deg(identity,target),)))
    print('559,'+fmt((geodesic_deg(z,x),)))
    print('error,'+fmt((geodesic_deg(axis_deg((0,0,1),88),z),)))
    end = axis_deg((0,0,1),170)
    with open('angles.csv','w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(['t','slerp_deg','nlerp_deg'])
        for i in range(101):
            t=i/100
            writer.writerow([f'{t:.4f}',f'{geodesic_deg(identity,slerp(identity,end,t)):.4f}',f'{geodesic_deg(identity,nlerp(identity,end,t)):.4f}'])

if __name__ == '__main__':
    results()
