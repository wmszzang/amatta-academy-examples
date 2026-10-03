import csv,math,sys
from pathlib import Path
from transform import rigid2
from fk import fk2,dh_planar,matmul
from ik import ik2,wrap
from jacobian import velocity,manipulability
from mobile import diff_ik,mecanum_ik,mecanum_fk,odom,pure_pursuit,ackermann,bicycle_step
from path import arc_points,joint_lerp,trapezoid,cubic_approach
from calib import estimate_svd,estimate_closed
from camera import project,backproject
from rotation import rotation_zyx,euler_zyx
Q=[(0,0),(1.2,0),(1.2,.8),(0,.8)]
P=[(1.5,-.8),(2.4829824531,-.1117082764),(2.0241213041,.5436133591),(1.0411388509,-.1446783646)]

def run(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    def write(name,header,rows):
        with (out/name).open('w',encoding='utf-8',newline='') as f:
            w=csv.writer(f,lineterminator='\n');w.writerow(header)
            w.writerows([[f'{v:.4f}' if isinstance(v,float) else v for v in row] for row in rows])
    diff=diff_ik(.6,-.5,.04,.36); mec=mecanum_ik(.9,-.4,.25,.05,.30,.26)
    write('wheels.csv',['problem','wheel','linear_m_s','angular_rad_s'],[(261,'L',diff[0],diff[2]),(261,'R',diff[1],diff[3])]+[(483,n,w*.05,w) for n,w in zip(['FL','FR','RL','RR'],mec)])
    od=odom(120*2*math.pi*.04/1024,150*2*math.pi*.04/1024,.36,40)
    write('odom.csv',['step','x','y','theta'],[(i,*p) for i,p in enumerate(od)])
    k,delta=pure_pursuit((math.sqrt(9-.375**2),.375),2.7)
    angles=ackermann(2.7,1.6,12);pose=(0.,0.,0.);steer=[(0.,*pose,*angles)]
    for i in range(1,21):
        pose=bicycle_step(pose,1.5,delta,2.7,.1);steer.append((i*.1,*pose,*angles))
    write('steer.csv',['t','x','y','theta','delta_in','delta_out'],steer)
    arc=arc_points((1.2,.6),.5,math.radians(30),math.radians(150),6)
    sol=[ik2(p,(1.2,.9)) for p in arc]
    write('arc_path.csv',['i','angle','x','y','th1','th2','w','reachable'],[(i,30+20*i,*p,*(math.degrees(t) for t in q),manipulability(q,(1.2,.9)),1) for i,(p,q) in enumerate(zip(arc,sol))])
    gaps=[]
    for i,p in enumerate(arc):
        lerp=fk2(joint_lerp(sol[0],sol[-1],i/6),(1.2,.9));gaps.append((i,*lerp,*p,math.dist(lerp,p)))
    write('jointlerp_gap.csv',['i','x_lerp','y_lerp','x_arc','y_arc','gap'],gaps)
    a,t,r,reflection=estimate_svd(Q,P);ac,tc=estimate_closed(Q,P)
    err=max(math.dist(rigid2(q,a,t),p) for q,p in zip(Q,P))
    (out/'calib.txt').write_text(f'SVD {math.degrees(a):.4f} {t[0]:.4f} {t[1]:.4f} {err:.4f}\nCLOSED {math.degrees(ac):.4f} {tc[0]:.4f} {tc[1]:.4f}\n',encoding='utf-8')
    uv=project((.36,-.15,3.),600,320,240);xyz=backproject(392,210,368,600,.12,320,240)
    write('camera.csv',['u','v','X','Y','Z'],[(*uv,*xyz)])
    dh=matmul(dh_planar(.4,2),dh_planar(.9,1.5));pos=(dh[0][2],dh[1][2])
    down=tuple(math.degrees(wrap(t)) for t in ik2((1.2,.7),(1,1)))
    vel=velocity((.3,.6),(.5,-.2),(2,1.5));euler=euler_zyx(rotation_zyx(-.4,.7,1.1))
    write('rejudge.csv',['id','a','b','c'],[(237,*pos,0.),(222,*down,0.),(239,*vel,manipulability((.3,.6),(2,1.5))),(252,*euler)])
    print('diff',*(f'{v:.4f}' for v in diff));print('mecanum',*(f'{v:.4f}' for v in mec))
    print('odom',*(f'{v:.4f}' for v in od[-1]),f'{math.degrees(od[-1][2]):.4f} deg')
    print('bicycle',*(f'{v:.4f}' for v in pose));print('SVD',f'{math.degrees(a):.4f}',*(f'{v:.4f}' for v in t))
    print('camera',uv,xyz);print('max sampled gap',f'{max(g[-1] for g in gaps):.4f}')
    return {'diff':diff,'mecanum':mec,'odom':od[-1],'bicycle':pose,'svd':(math.degrees(a),*t),'camera':(*uv,*xyz),'max_gap':max(g[-1] for g in gaps)}

if __name__=='__main__': run(sys.argv[1] if len(sys.argv)>1 else 'out')
