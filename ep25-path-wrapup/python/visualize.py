"""실제 계산 결과로 PNG와 12fps 프레임 시퀀스를 만든다."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from path_wrapup import sampling, verify, SAMPLES

def draw(count, output):
    nodes, path, _ = sampling(SAMPLES[:count])
    fig, ax = plt.subplots(figsize=(12, 7), dpi=100)
    fig.patch.set_facecolor('#f4f6fb')
    ax.set_facecolor('#f4f6fb')
    ax.add_patch(Circle((2,0),1,color='#ffe4e6',ec='#e11d48'))
    for i,node in enumerate(nodes):
        x,y = node['point']
        if node['parent'] >= 0:
            px,py = nodes[node['parent']]['point']
            ax.plot([px,x],[py,y],color='#a5b4fc',lw=2)
        ax.plot(x,y,'o',color='#4f46e5')
        ax.annotate(f'{node["name"]} {node["cost"]:.4f}',(x,y),xytext=(8,8),textcoords='offset points',fontsize=12)
    if path:
        ax.plot([nodes[i]['point'][0] for i in path],[nodes[i]['point'][1] for i in path],color='#4f46e5',lw=4)
    ax.set(xlim=(-1,5.4),ylim=(-1.5,3),xlabel='x [m]',ylabel='y [m]',title='Fixed samples: RRT* parent selection and rewiring')
    ax.set_aspect('equal');ax.grid(alpha=.15);fig.tight_layout();fig.savefig(output);plt.close(fig)

if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--frames',action='store_true');args=parser.parse_args()
    verify();draw(7,'path_wrapup.png')
    if args.frames:
        Path('frames').mkdir(exist_ok=True)
        for frame in range(84):
            draw(min(7,frame//12+1),Path('frames')/f'{frame:04d}.png')
