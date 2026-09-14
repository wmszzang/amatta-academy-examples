"""EP.11 계산 결과를 그림 또는 12fps 프레임으로 저장한다."""
import argparse
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from jacobian import jacobian, position, sweep_row, wrist_det

COLORS = ['#4f46e5', '#0d9488', '#e11d48']


def draw(mode, phase, destination):
    fig = plt.figure(figsize=(12, 6.75), dpi=100, facecolor='#f4f6fb')
    if mode == 'arrows':
        ax = fig.add_subplot(111)
        t1, t2 = .5236, .7854*(1-phase)
        p = np.array(position(2,1.5,t1,t2))
        elbow = np.array([2*math.cos(t1),2*math.sin(t1)])
        j = np.array(jacobian(2,1.5,t1,t2))
        ax.plot([0,elbow[0],p[0]],[0,elbow[1],p[1]],'o-',color='#334155',lw=7)
        origin = p
        c1,c2 = j[:,0]*.45,j[:,1]*.45
        poly=np.array([origin,origin+c1,origin+c1+c2,origin+c2])
        ax.fill(poly[:,0],poly[:,1],color=COLORS[0],alpha=.14)
        for c,color in zip([c1,c2],COLORS):
            ax.arrow(*origin,*c,width=.025,color=color,length_includes_head=True)
        ax.set(xlim=(-1,5),ylim=(-.5,4.2),xlabel='x (m)',ylabel='y (m)',
               title=f'Jacobian columns: visual scale 0.45 | det J = {np.linalg.det(j):.4f}')
        ax.set_aspect('equal')
    elif mode == 'sweep':
        ax = fig.add_subplot(121)
        bx = fig.add_subplot(122)
        ts = np.geomspace(.001,.7854,160)
        rows=np.array([sweep_row(t) for t in ts])
        ax.plot(ts,rows[:,1],color=COLORS[0],lw=3)
        bx.semilogy(ts,rows[:,4],color=COLORS[1],lw=3)
        t=.7854*(.001/.7854)**phase
        row=sweep_row(t)
        ax.scatter([t],[row[1]],color=COLORS[2],s=70)
        bx.scatter([t],[row[4]],color=COLORS[2],s=70)
        ax.set(xlabel='theta2 (rad)',ylabel='det J',title='Area factor')
        bx.set(xlabel='theta2 (rad)',ylabel='joint speed norm (rad/s)',title='Radial command: 0.10 m/s')
        bx.grid(alpha=.2)
    else:
        ax=fig.add_subplot(121,projection='3d')
        bx=fig.add_subplot(122)
        beta=.7854*(1-phase)
        axes=[(0,0,1),(0,1,0),(math.sin(beta),0,math.cos(beta))]
        for z,color in zip(axes,COLORS):
            ax.quiver(0,0,0,*z,color=color,linewidth=3,length=1)
        ax.set(xlim=(-.1,1),ylim=(-.1,1),zlim=(0,1.1),xlabel='x',ylabel='y',zlabel='z',title=f'beta = {beta:.4f}')
        bs=np.linspace(0,.7854,80)
        bx.plot(bs,np.sin(bs),color=COLORS[0],lw=3)
        bx.scatter([beta],[wrist_det(beta)],color=COLORS[2],s=65)
        bx.set(xlabel='beta (rad)',ylabel='abs(det Jw)',title='Common frame: three wrist axes')
    fig.tight_layout(pad=2.5)
    fig.savefig(destination)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    for name in ('arrows','sweep','wrist','frames'):
        parser.add_argument('--'+name,action='store_true')
    parser.add_argument('--out',default='.')
    args=parser.parse_args()
    out=Path(args.out)
    out.mkdir(parents=True,exist_ok=True)
    modes=[m for m in ('arrows','sweep','wrist') if getattr(args,m)] or ['arrows','sweep','wrist']
    for mode in modes:
        draw(mode,0,out/(mode+'.png'))
        if args.frames:
            frames=out/('frames-'+mode)
            frames.mkdir(exist_ok=True)
            for i in range(36):
                draw(mode,i/35,frames/f'frame_{i:03d}.png')
        print(f'{mode}: saved'+(' 36 frames at 12 fps' if args.frames else ''))


if __name__=='__main__':
    main()
