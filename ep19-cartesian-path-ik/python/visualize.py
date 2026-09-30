"""같은 계산 커널의 정적 그림과 15fps, 12초 프레임을 생성한다."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kin_path import P0,P1,L1,L2,ik2,fk2,trap_s,deviation,manip,path_ik_trap


def save_plot(filename='result.png', progress=1.):
    q0,q1=np.array(ik2(*P0)),np.array(ik2(*P1))
    u=np.linspace(0,1,201)
    joint=np.array([fk2(*(q0+t*(q1-q0))) for t in u])
    cart=np.array([np.array(P0)+t*(np.array(P1)-P0) for t in u])
    q=np.degrees([ik2(*p) for p in cart])
    ts=np.linspace(0,2,201); L=np.linalg.norm(np.array(P1)-P0)
    ss=np.array([trap_s(t,L,2,.5) for t in ts])
    rows=np.array(path_ik_trap(P0,P1,2,.5,.1),dtype=float)
    fig,axs=plt.subplots(2,2,figsize=(12,8),facecolor='#f4f6fb',layout='constrained')
    ax=axs[0,0];ax.plot(*joint.T,color='#4f46e5',label='joint');ax.plot(*cart.T,color='#059669',label='Cartesian')
    k=min(200,round(progress*200));ax.scatter(*joint[k],color='#4f46e5');ax.scatter(*cart[k],color='#059669');ax.set_aspect('equal');ax.set(xlabel='x (m)',ylabel='y (m)',title='Same endpoints, different paths');ax.legend()
    ax=axs[0,1];ax.plot(u,q[:,0],label='joint 1');ax.plot(u,q[:,1],label='joint 2');ax.axvline(progress,color='#e84040');ax.set(xlabel='u',ylabel='angle (deg)',title='Per-point inverse kinematics');ax.legend()
    a=np.linspace(np.pi/6,2*np.pi/3,7);c=np.array([L1*np.cos(np.pi/6),L1*np.sin(np.pi/6)])
    arc=c+L2*np.column_stack((np.cos(a),np.sin(a)));sweep=np.array([fk2(np.pi/6,t) for t in np.linspace(0,np.pi/2,7)])
    ax=axs[1,0];ax.plot(*arc.T,'o-',label='arc');ax.plot(*sweep.T,'x',markersize=10,label='joint sweep');ax.set_aspect('equal');ax.set(xlabel='x (m)',ylabel='y (m)',title='Fixed first joint: coincident samples');ax.legend()
    ax=axs[1,1];ax.plot(ts,ss/L,label='s/L');ax.plot(rows[:,0],rows[:,8],label='w (m^2)');ax.plot(rows[1:,0],rows[1:,6]/50,label='avg speed 1 / 50 (deg/s)');ax.axvline(2*progress,color='#e84040');ax.set(xlabel='time (s)',title='Trapezoid distance, interval speed, manipulability');ax.legend(fontsize=8)
    for ax in axs.flat:ax.grid(alpha=.2)
    fig.savefig(filename,dpi=120);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--frames',action='store_true');args=parser.parse_args()
    save_plot()
    if args.frames:
        Path('frames').mkdir(exist_ok=True)
        for i in range(180):save_plot(f'frames/{i:04d}.png',i/179)
        print('frames=180; fps=15; duration=12 s')
