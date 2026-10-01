"""계산 결과로 정적 그림과 12fps 재생용 프레임을 만든다."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mobile_kin import odometry, tick_to_dist, verify


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--frames',action='store_true')
    parser.add_argument('--out',type=Path,default=Path('output'))
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    verify()
    pts,th=odometry(.1,.12,.5,40)
    slip,_=odometry(.1,.1164,.5,40)
    old,_=odometry(.1,.12,.5,40,False)
    fig,ax=plt.subplots(figsize=(12,6.75),dpi=120)
    fig.patch.set_facecolor('#F4F6FB')
    for path,label,color in [(pts,'Midpoint estimate','#4F46E5'),(slip,'Assumed 3% right slip','#E11D48'),(old,'Start-angle integration','#D97706')]:
        ax.plot(*zip(*path),label=label,color=color,linewidth=2.4)
    ax.set(xlabel='x (m)',ylabel='y (m)',xlim=(-.2,3.5),ylim=(-.2,3.5))
    ax.set_aspect('equal'); ax.grid(alpha=.25); ax.legend(loc='upper left')
    fig.tight_layout(); fig.savefig(args.out/'trajectory.png');plt.close(fig)
    vals=list(tick_to_dist([0,100,205,300,412],.05,2048,.1))
    fig,axes=plt.subplots(2,1,figsize=(12,6.75),dpi=120,sharex=True)
    for i,(label,color) in enumerate([('Cumulative distance (m)','#4F46E5'),('Velocity (m/s)','#059669')]):
        axes[i].plot([.1,.2,.3,.4,.5],[v[i] for v in vals],'o-',color=color)
        axes[i].set_ylabel(label);axes[i].grid(alpha=.25)
    axes[-1].set_xlabel('Time (s)');fig.tight_layout();fig.savefig(args.out/'encoder.png');plt.close(fig)
    if args.frames:
        folder=args.out/'frames';folder.mkdir(exist_ok=True)
        for step in range(1,41):
            fig,ax=plt.subplots(figsize=(12,6.75),dpi=120)
            fig.patch.set_facecolor('#F4F6FB')
            ax.plot(*zip(*pts),color='#CBD5E1',linewidth=2)
            ax.plot(*zip(*pts[:step+1]),color='#4F46E5',linewidth=3)
            ax.scatter(*pts[step],s=65,color='#E11D48')
            ax.set(xlabel='x (m)',ylabel='y (m)',xlim=(-.2,3.4),ylim=(-.2,3.4),title=f'Step {step:02d} / 40')
            ax.set_aspect('equal');ax.grid(alpha=.25)
            # 글자와 눈금은 아래 25%를 비워 휴대폰 CC와 겹치지 않게 한다.
            fig.subplots_adjust(left=.12,right=.88,bottom=.31,top=.85)
            fig.savefig(folder/f'{step:04d}.png');plt.close(fig)
    print(f'PASS: trajectory, encoder and frames={args.frames}; final=({pts[-1][0]:.4f},{pts[-1][1]:.4f},{th:.4f})')


if __name__=='__main__':
    main()
