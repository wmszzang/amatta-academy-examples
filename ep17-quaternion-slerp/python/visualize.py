"""실제 계산값으로 정적 그림과 12fps 시퀀스를 만듭니다."""
import argparse
from pathlib import Path
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from quat import axis_deg, rotate
from slerp import slerp, nlerp
from distance import geodesic_deg

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':15,
                     'text.color':'#1E293B','axes.labelcolor':'#1E293B',
                     'figure.facecolor':'#F4F6FB','axes.facecolor':'#F4F6FB'})
I=(1,0,0,0)
E=axis_deg((0,0,1),170)
ts=np.linspace(0,1,101)
sa=[geodesic_deg(I,slerp(I,E,t)) for t in ts]
la=[geodesic_deg(I,nlerp(I,E,t)) for t in ts]

def draw_arc(t,output):
    fig=plt.figure(figsize=(12,6.75),dpi=80)
    left=fig.add_subplot(121,projection='3d')
    u,v=np.linspace(0,2*np.pi,25),np.linspace(0,np.pi,15)
    left.plot_wireframe(np.outer(np.cos(u),np.sin(v)),np.outer(np.sin(u),np.sin(v)),
                        np.outer(np.ones_like(u),np.cos(v)),color='#CBD5E1',linewidth=.5)
    angles=np.linspace(0,math.radians(85),70)
    left.plot(np.cos(angles),np.sin(angles),np.zeros_like(angles),color='#4F46E5',lw=4)
    q=slerp(I,E,t)
    left.scatter(q[0],q[3],0,color='#E11D48',s=80)
    left.set(xlabel='w',ylabel='z',zlabel='Other component',title='Unit-quaternion arc (section)')
    left.set_box_aspect((1,1,1));left.view_init(25,-50)
    right=fig.add_subplot(122,projection='3d')
    for basis,color,name in zip(np.eye(3),['#E11D48','#059669','#0284C7'],['x','y','z']):
        vector=rotate(q,basis)
        right.quiver(0,0,0,*vector,color=color,linewidth=3,arrow_length_ratio=.14)
        right.text(*(np.array(vector)*1.13),name,color=color)
    right.set(xlim=(-1.3,1.3),ylim=(-1.3,1.3),zlim=(-1.3,1.3),title=f'Object axes: {170*t:.1f} degrees')
    right.set_box_aspect((1,1,1));right.view_init(25,-50)
    fig.suptitle(f'Progress t = {t:.2f}  |  physical angle = 2 x quaternion angle',fontsize=18)
    fig.subplots_adjust(top=.82,left=.03,right=.96,bottom=.06,wspace=.12)
    fig.savefig(output);plt.close(fig)

def draw_angles(t,output):
    fig,ax=plt.subplots(figsize=(12,6.75),dpi=80)
    ax.plot(ts,sa,color='#4F46E5',lw=4,label='SLERP')
    ax.plot(ts,la,color='#D97706',lw=4,label='Normalized LERP')
    for f,color in [(slerp,'#4F46E5'),(nlerp,'#D97706')]:
        angle=geodesic_deg(I,f(I,E,t));ax.scatter(t,angle,color=color,s=100,zorder=3)
    ax.axvline(t,color='#64748B',linestyle='--')
    ax.set(xlim=(0,1),ylim=(0,175),xlabel='Progress t (time-proportional)',ylabel='Angle from start (degrees)',
           title=f'170-degree rotation | t={t:.2f}')
    ax.grid(alpha=.2);ax.legend(loc='upper left');fig.tight_layout(pad=2)
    fig.savefig(output);plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--frames',action='store_true')
    args=parser.parse_args()
    draw_arc(.25,'arc.png');draw_angles(.25,'angles.png')
    if args.frames:
        for name,draw in [('arc',draw_arc),('angles',draw_angles)]:
            folder=Path('frames')/name;folder.mkdir(parents=True,exist_ok=True)
            for i in range(97):
                draw(i/96,folder/f'frame_{i:03d}.png')
        print('frames: arc 97, angles 97, 12fps')
