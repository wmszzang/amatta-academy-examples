"""실제 CSV에서 세 시간 그래프와 같은 배율의 좌표 그림을 만든다."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def draw(directory, out, fraction=1.):
    directory, out = Path(directory), Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with (directory/'trajectory.csv').open(encoding='utf-8') as f:
        rows=np.array([[float(x) for x in row] for row in list(csv.reader(f))[1:]])
    with (directory/'points.csv').open(encoding='utf-8') as f:
        points=np.array([[float(x) for x in row[1:]] for row in list(csv.reader(f))[1:]])
    fig=plt.figure(figsize=(16,9),dpi=120,facecolor='#F4F6FB')
    grid=fig.add_gridspec(3,2,width_ratios=[1.3,1],hspace=.7,wspace=.25)
    count=max(1,int(fraction*(len(rows)-1))+1)
    for i,(label,color,limits) in enumerate([('x (m)','#4F46E5',(0,11)),('v (m/s)','#0284C7',(0,2.4)),('a (m/s²)','#059669',(-1.4,1.4))]):
        ax=fig.add_subplot(grid[i,0]); ax.set_facecolor('white')
        ax.plot(rows[:count,0],rows[:count,i+1],color=color,marker='.',drawstyle='steps-post' if i==2 else 'default')
        ax.set(xlim=(0,7),ylim=limits,xlabel='t (s)',ylabel=label)
        ax.grid(alpha=.2)
    ax=fig.add_subplot(grid[:,1]); ax.set_facecolor('white')
    for cols,color,label in [([0,1],'#94A3B8','Original'),([2,3],'#4F46E5','Rotated then translated')]:
        poly=points[:,cols]; poly=np.vstack((poly,poly[0]))
        ax.plot(poly[:,0],poly[:,1],'-o',color=color,label=label)
    for name,(x,y) in zip('ABCD',points[:,2:]): ax.annotate(name,(x,y),xytext=(8,8),textcoords='offset points')
    ax.set(xlim=(-.4,2.3),ylim=(-.4,2.3),xlabel='x',ylabel='y')
    ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='upper left',fontsize=9)
    fig.suptitle('EP.29 | #119 motion profile + #226 rigid transform',fontsize=21,color='#172033')
    fig.subplots_adjust(left=.09,right=.95,top=.82,bottom=.31)
    fig.savefig(out,facecolor=fig.get_facecolor()); plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--data',default='.')
    parser.add_argument('--out',default='result.png')
    parser.add_argument('--frames',action='store_true')
    args=parser.parse_args()
    draw(args.data,args.out)
    if args.frames:
        for i in range(36): draw(args.data,Path(args.out).parent/'frames'/f'frame_{i:03d}.png',i/35)
