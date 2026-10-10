"""실제 계산 결과의 정적 그림과 12fps 프레임을 만든다."""
from pathlib import Path
import argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from control_wrapup import results, verify


def draw(out, progress=1.):
    data=results()
    fig, axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    fig.patch.set_facecolor('#f4f6fb')
    plant=data['plant'][1]
    stop=max(1,round(200*progress))
    axs[0,0].plot([0]+[r[3] for r in plant[:stop]],[0]+[r[4] for r in plant[:stop]],color='#4f46e5',label='Plant output')
    axs[0,0].axhline(1,color='#e11d48',ls='--',label='Target')
    axs[0,0].set(xlim=(0,10),ylim=(0,1.1),xlabel='Time (s)',ylabel='Output',title='#245 PID + first-order plant')
    for key,color in [('hold','#4f46e5'),('no_hold','#e11d48')]:
        rows=data[key][1]
        axs[0,1].plot([0]+[r[0] for r in rows],[0]+[r[6] for r in rows],marker='o',color=color,label=key)
    axs[0,1].set(xlabel='Step',ylabel='Stored integral term',title='#500 Stored integral')
    rows=data['kalman'][1]
    axs[1,0].plot([1,2],[r[1] for r in rows],'o--',label='Predicted')
    axs[1,0].plot([1,2],[r[5] for r in rows],'o-',label='Corrected')
    axs[1,0].scatter([1,2],[1.2,2.1],marker='x',label='Measured')
    axs[1,0].set(xlabel='Step',ylabel='Position',title='#399 Kalman positions')
    rows=data['complementary'][1][:max(1,round(20*progress))]
    for column,label,color in [(5,'Gyro only','#e11d48'),(3,'Example truth','#0284c7'),(4,'Complementary','#4f46e5')]:
        axs[1,1].plot([0]+[r[1] for r in rows],[0]+[r[column] for r in rows],color=color,label=label)
    axs[1,1].set(xlim=(0,2),ylim=(0,25),xlabel='Time (s)',ylabel='Angle (deg)',title='#228 Complementary filter')
    for ax in axs.flat:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(out,dpi=100)
    plt.close(fig)


def draw_frame(out,progress):
    rows=results()['plant'][1][:max(1,round(200*progress))]
    fig=plt.figure(figsize=(16,9),facecolor='#f4f6fb')
    ax=fig.add_axes([.15,.32,.70,.42])
    ax.plot([0]+[r[3] for r in rows],[0]+[r[4] for r in rows],color='#4f46e5',lw=3)
    ax.axhline(1,color='#e11d48',ls='--')
    ax.set(xlim=(0,10),ylim=(0,1.1))
    ax.set_xlabel('Time (s)',fontsize=20)
    ax.set_ylabel('Plant output',fontsize=20)
    ax.tick_params(labelsize=18)
    ax.grid(alpha=.2)
    fig.savefig(out,dpi=100)
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--frames',action='store_true')
    args=parser.parse_args()
    verify()
    draw('control-wrapup.png')
    if args.frames:
        Path('frames').mkdir(exist_ok=True)
        for k in range(48):
            draw_frame(Path('frames')/f'{k:04d}.png',(k+1)/48)
    print('Verified calculations: control-wrapup.png'+(' + 48 frames at 12 fps' if args.frames else ''))
