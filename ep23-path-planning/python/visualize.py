"""검산과 같은 입력에서 정적 그림과 12fps 탐색 프레임을 만든다."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from path_planning import search, TERRAIN, GRID, point_segment

def draw(out, upto=None):
    fig, axes=plt.subplots(1,3,figsize=(16,6),facecolor='#F4F6FB')
    for ax,a,terrain,title in zip(axes[:2],(TERRAIN,GRID),(True,False),('Dijkstra: cost 7','A*: 7 points / 6 moves')):
        cost,path,trace=search(a,terrain=terrain)
        for r,row in enumerate(a):
            for c,value in enumerate(row):
                blocked=not terrain and value==1
                ax.add_patch(Rectangle((c-.5,r-.5),1,1,facecolor='#334155' if blocked else 'white',edgecolor='#94A3B8'))
                ax.text(c,r,str(value),ha='center',va='center',color='white' if blocked else '#1E293B')
        count=len(trace) if upto is None else min(upto,len(trace))
        for r,c,*_ in trace[:count]:ax.add_patch(Rectangle((c-.45,r-.45),.9,.9,fill=False,edgecolor='#D97706',linewidth=3))
        if count==len(trace):ax.plot([c for r,c in path],[r for r,c in path],color='#4F46E5',linewidth=3,marker='o')
        ax.set(xlim=(-.8,len(a[0])-.2),ylim=(len(a)-.2,-.8),title=title,xlabel='column',ylabel='row');ax.set_aspect('equal')
    ax=axes[2];ax.plot([0,4],[0,0],color='#4F46E5',linewidth=4)
    for p in ((2,3),(6,0),(-1,-1)):
        d,q,raw,t=point_segment((0,0),(4,0),p)
        ax.plot([p[0],q[0]],[p[1],q[1]],'o--',label=f'd={d:.4f}')
    ax.add_patch(Circle((2,1),1,fill=False,color='#E11D48'))
    ax.set(xlim=(-1.8,6.8),ylim=(-1.8,4),title='Point / segment / touching circle',xlabel='x',ylabel='y');ax.set_aspect('equal');ax.legend()
    fig.tight_layout();fig.savefig(out,dpi=90);plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--frames',action='store_true');parser.add_argument('--out',default='visualization.png');args=parser.parse_args()
    draw(args.out)
    if args.frames:
        Path('frames').mkdir(exist_ok=True)
        for i in range(108):draw(Path('frames')/f'{i:04d}.png',min(8,i//12+1))
