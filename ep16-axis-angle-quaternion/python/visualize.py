"""검산 함수로 계산한 실제 3차원 회전 프레임을 저장한다."""
import argparse
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from axis_angle_quat import rodrigues


def draw(theta, mode, out):
    k=np.array([1,2,2])/3
    fig=plt.figure(figsize=(9.6,5.4),dpi=100,facecolor='#F4F6FB')
    ax=fig.add_subplot(111,projection='3d',facecolor='#F4F6FB')
    ax.set(xlim=(-1.2,1.2),ylim=(-1.2,1.2),zlim=(-1.2,1.2),xlabel='x',ylabel='y',zlabel='z')
    ax.set_box_aspect([1,1,1])
    ax.view_init(elev=23,azim=35)
    trajectory=np.array([np.array(rodrigues(k,a))[:,0] for a in np.linspace(0,2*math.pi,121)])
    ax.plot(*trajectory.T,color='#94A3B8',lw=2)
    if mode!='doublecover':
        for endpoint in trajectory[::8]:
            ax.plot([0,endpoint[0]],[0,endpoint[1]],[0,endpoint[2]],alpha=.2,color='#4F46E5')
    R=np.array(rodrigues(k,theta))
    axis=-k if mode=='doublecover' else k
    ax.quiver(0,0,0,*axis,length=1.3,color='#64748B',linewidth=3)
    ax.quiver(0,0,0,1,0,0,color='#D97706',linewidth=2)
    if mode!='doublecover':
        ax.quiver(0,0,0,*R[:,0],color='#4F46E5',linewidth=4)
    if mode=='doublecover':
        positive=np.array(rodrigues(k,math.pi/3))
        negative=np.array(rodrigues(-k,theta))
        for i,c in enumerate(['#DC2626','#059669','#0284C7']):
            ax.plot([0,positive[0,i]],[0,positive[1,i]],[0,positive[2,i]],color=c,lw=5,alpha=.3)
            ax.quiver(0,0,0,*negative[:,i],color=c,linewidth=2)
        title=f'fixed +k: 60 deg | moving -k: {math.degrees(theta):.0f} deg'
    else:
        title=f'+k=(1,2,2)/3 | angle={math.degrees(theta):.0f} deg'
    ax.set_title(title,color='#1E293B',fontsize=15,pad=8)
    fig.subplots_adjust(left=0,right=1,bottom=.07,top=.9)
    fig.savefig(out,dpi=100)
    plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group()
    group.add_argument('--cone',action='store_true')
    group.add_argument('--sweep',action='store_true')
    group.add_argument('--doublecover',action='store_true')
    p.add_argument('--frames',action='store_true')
    p.add_argument('--out',default='frames')
    args=p.parse_args()
    mode='doublecover' if args.doublecover else ('sweep' if args.sweep else 'cone')
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    if args.frames:
        angles=np.linspace(0,5*math.pi/3,61) if mode=='doublecover' else np.r_[np.linspace(0,math.pi/3,25),np.full(12,math.pi/3),np.linspace(math.pi/3,2*math.pi,61)]
        for i,angle in enumerate(angles):
            draw(angle,mode,out/f'{i:04d}.png')
        print(f'{mode}: {len(angles)} frames, 960x540')
    else:
        draw(5*math.pi/3 if mode=='doublecover' else math.pi/3,mode,out/'still.png')
        print(out/'still.png')


if __name__=='__main__':
    main()
