"""실행 결과를 정적 PNG 또는 12 fps 프레임으로 확인한다."""
import argparse
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from workspace import sweep, manip, cond
from kin_common import fk, jacobian
from hull_area import convex_hull
from cspace import cspace_grid

MODES = ('annulus', 'limits', 'manip', 'ellipse', 'hull', 'cspace')

def render(mode, out, frames=False):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    limited = np.array(list(sweep()))
    q1, q2 = np.meshgrid(np.linspace(-math.pi, math.pi, 121), np.linspace(-math.pi, math.pi, 101))
    full = np.array([(*fk(a,b,2,1.5),manip(a,b)) for a,b in zip(q1.ravel(),q2.ravel())])
    hull = np.array(convex_hull(limited[:,:2]))
    free, hit = cspace_grid()
    for frame in range(48 if frames else 1):
        progress = (frame + 1)/48 if frames else 1
        fig = plt.figure(figsize=(12,6.75),dpi=100,facecolor='#F4F6FB')
        ax = fig.add_subplot(121 if mode=='cspace' else 111)
        ax.set_facecolor('#F4F6FB')
        ax.set_aspect('equal');ax.set_xlim(-4,4);ax.set_ylim(-3.7,3.7)
        ax.set_xlabel('x');ax.set_ylabel('y');ax.grid(alpha=.2)
        ax.set_title(mode.title(),fontsize=19)
        if mode in ('annulus','limits','manip','hull'):
            data=full if mode in ('annulus','manip') else limited
            n=max(1,int(len(data)*progress))
            sc=ax.scatter(data[:n,0],data[:n,1],s=3,c=data[:n,2] if mode=='manip' else '#4F46E5',cmap='viridis' if mode=='manip' else None,vmin=0 if mode=='manip' else None,vmax=3 if mode=='manip' else None)
            if mode=='manip':fig.colorbar(sc,ax=ax,label='w (area scale)')
            if mode=='hull':
                h=np.vstack([hull,hull[0]]);count=max(2,int(len(h)*progress));ax.plot(h[:count,0],h[:count,1],color='#E11D48',linewidth=2)
        if mode=='ellipse':
            a=math.radians(25);b=math.radians(10+100*progress)
            end=fk(a,b,2,1.5);el=(2*math.cos(a),2*math.sin(a))
            ax.plot([0,el[0],end[0]],[0,el[1],end[1]],'o-',linewidth=4,color='#4F46E5')
            unit=np.array([np.cos(np.linspace(0,2*math.pi,120)),np.sin(np.linspace(0,2*math.pi,120))])
            # 팔과 함께 보이도록 타원의 표시 배율만 줄인다.
            ellipse=np.array(jacobian(2,1.5,a,b))@unit*.35
            ax.plot(ellipse[0]+end[0],ellipse[1]+end[1],color='#059669')
            ax.set_title(f'w={manip(a,b):.4f}, kappa={cond(a,b):.4f} (ellipse display x0.35)')
        if mode=='cspace':
            a=math.radians(-180+360*progress);b=math.radians(50)
            el=(2*math.cos(a),2*math.sin(a));end=fk(a,b,2,1.5)
            ax.plot([0,el[0],end[0]],[0,el[1],end[1]],'o-',linewidth=4,color='#4F46E5')
            ax.add_patch(plt.Circle((1.6,1.2),.35,color='#E11D48',alpha=.6))
            other=fig.add_subplot(122);other.set_facecolor('#F4F6FB')
            other.scatter(*np.array(free).T,s=3,color='#4F46E5');other.scatter(*np.array(hit).T,s=5,color='#E11D48')
            other.scatter([math.degrees(a)],[50],s=80,color='#D97706',edgecolor='black')
            other.set(xlim=(-180,180),ylim=(-180,180),xlabel='theta1 (deg)',ylabel='theta2 (deg)',title='C-space: 452 obstacle / 4732 free')
        fig.tight_layout()
        fig.savefig(out/(f'{frame:04d}.png' if frames else f'{mode}.png'))
        plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for mode in MODES:parser.add_argument('--'+mode,action='store_true')
    parser.add_argument('--frames',action='store_true');parser.add_argument('--out',default='figures')
    args=parser.parse_args()
    selected=[m for m in MODES if getattr(args,m)] or list(MODES)
    for mode in selected:
        render(mode,Path(args.out)/mode,args.frames)
        print(mode, '48 frames' if args.frames else 'PNG')
