import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from filters import datasets

def draw(rows, stop, out):
    fig, ax = plt.subplots(figsize=(12,6.75), facecolor='#F4F6FB')
    ax.set_facecolor('#F4F6FB')
    used=rows[:stop]
    ax.plot([0]+[r[0] for r in used],[0]+[r[7] for r in used],'-o',color='#4F46E5',label='Estimate (initial = 0)')
    ax.scatter([r[0] for r in used],[r[2] for r in used],color='#D97706',label='Measurement')
    ax.axhline(5,color='#059669',linestyle='--',label='Reference truth (not input)')
    ax.set(xlim=(0,15.5),ylim=(-0.2,7),xlabel='Measurement index',ylabel='Position (problem units)')
    ax.legend(loc='lower right'); ax.grid(alpha=.2); fig.tight_layout()
    fig.savefig(out,dpi=120); plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--frames',action='store_true');a=p.parse_args()
    rows=datasets()['stationary'];draw(rows,15,'stationary.png')
    moving=datasets()['moving'];fig,ax=plt.subplots(figsize=(12,6.75))
    for col,label in [(3,'Prediction'),(2,'Measurement'),(7,'Correction')]:
        ax.plot([0,1,2],[0]+[r[col] for r in moving],'-o',label=label)
    ax.set(xlabel='Measurement index',ylabel='Position (problem units)');ax.legend();ax.grid(alpha=.2)
    fig.savefig('moving.png',dpi=120);plt.close(fig)
    angles=datasets()['complementary'];fig,ax=plt.subplots(figsize=(12,6.75))
    for col,label in [(2,'Gyro integration'),(3,'Accelerometer angle'),(5,'Complementary')]:
        ax.plot([0]+[r[1] for r in angles],[0]+[r[col] for r in angles],label=label)
    ax.set(xlabel='Time (s)',ylabel='Angle (deg)');ax.legend();ax.grid(alpha=.2)
    fig.savefig('complementary.png',dpi=120);plt.close(fig)
    if a.frames:
        Path('frames').mkdir(exist_ok=True)
        for i in range(16):draw(rows,i,Path('frames')/f'frame_{i:02}.png')
