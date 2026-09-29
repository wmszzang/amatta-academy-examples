"""같은 계산 입력의 2×2 그림. --frames는 15fps, 12초 PNG를 저장한다."""
import argparse
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rot import euler_zyx_to_R, rodrigues, q_to_R, R_to_q, unit, slerp, gyro_heading


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--frames',action='store_true')
    parser.add_argument('--out',default='output-visual')
    args=parser.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    if args.frames:(out/'frames').mkdir(exist_ok=True)
    eye=[1,0,0,0]
    end=R_to_q(euler_zyx_to_R(*map(math.radians,[90,80,90])))
    direction=np.array([1.,0,0])
    times=np.linspace(0,1,101)
    paths=[np.array([np.array(q_to_R(slerp(eye,end,t)))@direction for t in times]),
           np.array([np.array(q_to_R(unit([(1-t)*a+t*b for a,b in zip(eye,end)])))@direction for t in times]),
           np.array([np.array(euler_zyx_to_R(*[t*math.radians(a) for a in [90,80,90]]))@direction for t in times])]
    plt.rcParams.update({'font.size':11,'axes.facecolor':'#f4f6fb','figure.facecolor':'#f4f6fb'})
    for frame in range(180 if args.frames else 1):
        t=frame/179 if args.frames else 1
        fig=plt.figure(figsize=(12.8,7.2))
        axes=[fig.add_subplot(221+i,projection='3d' if i<3 else None) for i in range(4)]
        for ax in axes[:3]:
            ax.set(xlim=(-1.2,1.2),ylim=(-1.2,1.2),zlim=(-1.2,1.2),xlabel='x',ylabel='y',zlabel='z')
            ax.set_box_aspect([1,1,1]);ax.view_init(24,35)
        # 고정축 X, Y, Z 순차 적용은 내재 ZYX와 같은 행렬이다.
        target_angles=[.3,-.5,1.2]
        amounts=[angle*min(1,max(0,3*t-i)) for i,angle in enumerate(target_angles)]
        R=np.array(euler_zyx_to_R(*amounts))
        for i,c in enumerate(['#e11d48','#059669','#4f46e5']):
            axes[0].quiver(0,0,0,*R[:,i],color=c,length=.95)
        axes[0].set_title('Euler input: (0.3, -0.5, 1.2) rad')
        k=np.array(unit([1,2,2]));v=np.array(rodrigues(k,t))@direction
        axes[1].quiver(0,0,0,*k,color='#4f46e5')
        axes[1].quiver(0,0,0,*v,color='#d97706')
        cone=np.array([np.array(rodrigues(k,a))@direction for a in np.linspace(0,2*math.pi,100)])
        axes[1].plot(*cone.T,color='#c7d2fe');axes[1].set_title('Axis-angle: fixed axis, rotating direction')
        for path,color,label in zip(paths,['#4f46e5','#d97706','#e11d48'],['SLERP','normalized LERP','Euler component interpolation']):
            axes[2].plot(*path.T,color=color,label=label)
            axes[2].scatter(*path[round(t*100)],color=color,s=30)
        axes[2].legend(fontsize=7,loc='upper left');axes[2].set_title('Rotated unit direction (not quaternion space)')
        omega=[.1,.2,.2,.1];stamp=np.arange(4)*.5
        corrected=[gyro_heading(omega[:i+1],.5,.1)for i in range(4)]
        bad=[gyro_heading(omega[:i+1],.5,0)for i in range(4)]
        left=[sum(w-.1 for w in omega[:i])*.5 for i in range(4)]
        for values,color,label in [(corrected,'#4f46e5','trapezoid, bias removed'),(left,'#d97706','left rectangles, bias removed'),(bad,'#e11d48','bias retained')]:
            axes[3].plot(stamp,values,'o-',color=color,label=label)
        axes[3].axvline(t*1.5,color='#64748b',linestyle=':')
        axes[3].set(xlabel='time (s)',ylabel='heading (rad)',title='Fixed-axis gyro integration')
        axes[3].legend(fontsize=8);axes[3].grid(alpha=.2)
        fig.tight_layout(pad=2)
        fig.savefig(out/('frames/%04d.png'%frame if args.frames else 'rotation.png'),dpi=100)
        plt.close(fig)
    print('Saved',180 if args.frames else 1,'frames to',out)


if __name__=='__main__':main()
