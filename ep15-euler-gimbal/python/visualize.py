"""몸체의 직각축과 회전 동작의 축을 구분하는 실제 계산 시각화."""
import argparse
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from euler_gimbal import euler_to_R


def rotation(axis, angle):
    v = [0., 0., 0.]
    v[axis] = angle
    return np.array(euler_to_R(*v))


def draw(roll, pitch, yaw, order, path):
    Rz, Ry, Rx = rotation(2,yaw), rotation(1,pitch), rotation(0,roll)
    R = Rz@Ry@Rx if order == 'zyx' else Rx@Ry@Rz
    fig = plt.figure(figsize=(9.6,5.4), dpi=100, facecolor='#F4F6FB')
    colors = ['#E11D48','#059669','#4F46E5']
    action = [np.array([0.,0.,1.]), Rz@np.array([0.,1.,0.]), Rz@Ry@np.array([1.,0.,0.])]
    for number, title in [(1,'Body axes: always orthogonal'),(2,'ZYX action axes: first / third can align')]:
        ax = fig.add_subplot(1,2,number,projection='3d',facecolor='#F4F6FB')
        vectors = [R[:,i] for i in range(3)] if number==1 else action
        for i,v in enumerate(vectors):
            if number==1:
                ax.quiver(0,0,0,*v,color=colors[i],linewidth=3,arrow_length_ratio=.14)
                ax.text(*(v*1.1),['X','Y','Z'][i],color=colors[i],fontsize=12)
            else:
                ax.plot([-v[0],v[0]],[-v[1],v[1]],[-v[2],v[2]],
                        color=colors[i],linewidth=5-i,linestyle=['-','--',':'][i],
                        label=['1: yaw','2: pitch','3: roll'][i])
        ax.set(xlim=(-1.2,1.2),ylim=(-1.2,1.2),zlim=(-1.2,1.2),title=title)
        ax.set_box_aspect((1,1,1)); ax.view_init(elev=22,azim=38)
        ax.set_xticks([]);ax.set_yticks([]);ax.set_zticks([])
        if number==2: ax.legend(loc='lower left',fontsize=9)
    fig.suptitle(f'Intrinsic {order.upper()} | roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f} rad',fontsize=14)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--order',choices=['zyx','xyz'],default='zyx')
    parser.add_argument('--gimbal',action='store_true')
    parser.add_argument('--frames',action='store_true')
    parser.add_argument('--out',default='.')
    args=parser.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    draw(.2 if args.gimbal else .3,math.pi/2 if args.gimbal else -.5,
         .7 if args.gimbal else 1.2,args.order,out/'axes.png')
    if args.frames:
        frames=out/'frames';frames.mkdir(exist_ok=True)
        for i in range(73):
            t=i/72
            if args.gimbal: r,p,y=.2,(-.5+t)*math.pi,.7
            elif args.order=='zyx':
                y=1.2*min(1,t*3);p=-.5*min(1,max(0,t*3-1));r=.3*min(1,max(0,t*3-2))
            else:
                r=.3*min(1,t*3);p=-.5*min(1,max(0,t*3-1));y=1.2*min(1,max(0,t*3-2))
            draw(r,p,y,args.order,frames/f'frame_{i:03d}.png')
        print(f'{frames}: 73 frames, 960x540, 12 fps playback')


if __name__=='__main__':main()
