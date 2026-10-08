import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def draw(data, aw, output, upto=None):
    fig, axes=plt.subplots(2,2,figsize=(12,6.2),facecolor="#F4F6FB")
    n=upto or len(data)
    for ax in axes.flat:
        ax.set_facecolor("white")
        ax.grid(alpha=.2)
    axes[0,0].plot(data[:n,0],data[:n,1],color="#4F46E5",lw=3)
    axes[0,0].axhline(1,color="#D97706",ls="--",label="target")
    axes[0,0].set(ylabel="Position (problem scale)",xlim=(0,3),ylim=(0,1.2),title="#229 position")
    axes[1,0].plot(data[:n,0],data[:n,7],color="#0284C7",lw=2)
    axes[1,0].set(xlabel="Time (s)",ylabel="Control input",xlim=(0,3),ylim=(-.1,2.2))
    axes[0,1].plot(aw[:,0],aw[:,7],"o-",color="#4F46E5",lw=3)
    axes[0,1].axhline(15,color="#D97706",ls="--")
    axes[0,1].set(title="#500 example B",ylabel="Limited output",xticks=range(4),ylim=(-25,25))
    axes[1,1].step(aw[:,0],aw[:,8],where="mid",color="#059669",lw=3)
    axes[1,1].set(xlabel="Calculation index",ylabel="Stored integral term I",xticks=range(4),ylim=(-15,5))
    fig.tight_layout()
    fig.savefig(output,dpi=100)
    plt.close(fig)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--frames",action="store_true")
    args=parser.parse_args()
    data=np.loadtxt("position.csv",delimiter=",",skiprows=1)
    aw=np.loadtxt("B.csv",delimiter=",",skiprows=1)
    draw(data,aw,"pid.png")
    if args.frames:
        Path("frames").mkdir(exist_ok=True)
        for i in range(36):
            draw(data,aw,Path("frames")/f"frame_{i:03}.png",max(1,round(31*(i+1)/36)))
    print("pid.png" + (" + 36 frames (12 fps)" if args.frames else ""))
