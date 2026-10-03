"""실행 CSV에서 정적 그림과 12fps 프레임을 만든다."""
import argparse,csv,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kin import run,Q,P

def draw(folder,out,progress=1):
    def rows(name):
        with (folder/name).open(encoding='utf-8') as f:return list(csv.DictReader(f))
    od,steer,arc,gap=map(rows,['odom.csv','steer.csv','arc_path.csv','jointlerp_gap.csv'])
    fig,axes=plt.subplots(2,3,figsize=(16,9),facecolor='#f4f6fb')
    for ax in axes.flat:ax.grid(alpha=.2);ax.set_facecolor('white')
    def path(ax,data,x='x',y='y',label='',color='#4f46e5'):
        n=max(1,math.ceil(len(data)*progress));ax.plot([float(r[x]) for r in data[:n]],[float(r[y]) for r in data[:n]],'.-',color=color,label=label)
        ax.set(xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal',adjustable='datalim')
    path(axes[0,0],od);axes[0,0].set_title('Encoder odometry: midpoint')
    path(axes[0,1],steer);axes[0,1].set_title('Bicycle: forward Euler')
    path(axes[0,2],arc,label='Cartesian arc');path(axes[0,2],gap,'x_lerp','y_lerp','Joint LERP','#e11d48');axes[0,2].legend();axes[0,2].set_title('Seven sampled path points')
    a=math.radians(35)*progress
    moved=[(math.cos(a)*x-math.sin(a)*y+1.5*progress,math.sin(a)*x+math.cos(a)*y-.8*progress) for x,y in Q]
    ax=axes[1,0];ax.scatter(*zip(*P),label='Target P',marker='x',s=100);ax.scatter(*zip(*moved),label='Transformed Q',s=35);ax.legend();ax.set(xlabel='x (m)',ylabel='y (m)',title='SVD rigid registration');ax.set_aspect('equal',adjustable='datalim')
    ds=list(range(12,49));ax=axes[1,1];n=max(1,math.ceil(len(ds)*progress));ax.plot(ds[:n],[72/d for d in ds[:n]],color='#4f46e5');ax.set(xlabel='Disparity (px)',ylabel='Depth (m)',title='Stereo: Z = 72 / disparity',xlim=(10,50),ylim=(1,6.5))
    ax=axes[1,2];ax.axis('off');ax.set_title('Rejudge: output contract')
    table=rows('rejudge.csv');ax.table(cellText=[[r['id'],r['a'],r['b'],r['c']] for r in table],colLabels=['Problem','a','b','c'],loc='center',cellLoc='center').scale(1,2)
    fig.tight_layout(pad=2);fig.savefig(out,dpi=100);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--csv',default='out/python');p.add_argument('--frames',action='store_true');p.add_argument('--out',default='out');a=p.parse_args()
    folder=Path(a.csv);out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    if not (folder/'odom.csv').exists():run(folder)
    draw(folder,out/'overview.png')
    if a.frames:
        frames=out/'frames';frames.mkdir(exist_ok=True)
        for i in range(72):draw(folder,frames/f'{i:04d}.png',(i+1)/72)
    print('Wrote overview and requested frames from executed CSV')
