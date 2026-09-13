import argparse,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from ik2link import solve
p=argparse.ArgumentParser();p.add_argument('--frames',action='store_true');a=p.parse_args()
def draw(out,mode):
    fig,ax=plt.subplots(figsize=(10,6),dpi=120)
    ax.add_patch(Circle((0,0),3.5,color='#eef2ff'));ax.add_patch(Circle((0,0),.5,color='white'))
    for i,(_,t1,t2) in enumerate(solve(2.5,1.5,2,1.5)):
        e=(2*math.cos(t1),2*math.sin(t1))
        ax.plot([0,e[0],2.5],[0,e[1],1.5],'-o',lw=6,color=['#4f46e5','#059669'][i],alpha=1 if mode in (2,i) else .18,label=['down','up'][i])
    ax.plot(2.5,1.5,'x',ms=15,mew=3,color='#e11d48')
    ax.set(xlim=(-3.8,3.8),ylim=(-3.8,3.8),xlabel='x',ylabel='y',title='Two arrival configurations; target=(2.5, 1.5)')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(out);plt.close(fig)
draw('ik2link.png',2)
if a.frames:
    Path('frames').mkdir(exist_ok=True)
    for i in range(72):draw(f'frames/frame_{i:03}.png',(i//24)%3)
