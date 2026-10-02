import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from frame_chain import R_BC,T_BASE_CAM,apply_T
from icp_step import Q,P,estimate_rigid_2d
from calib_offset import example,fk
OUT=Path(__file__).resolve().parents[1]/"out"
OUT.mkdir(exist_ok=True)
plt.rcParams.update({"font.size":16,"axes.spines.top":False,"axes.spines.right":False})
def icp(frame=None):
    R,t=estimate_rigid_2d(Q,P)
    if frame is None: moved=Q@R.T+t
    else:
        a=frame/47
        if a<.33: moved=Q-Q.mean(0)*(a/.33)
        elif a<.66:
            angle=np.pi/2*(a-.33)/.33
            rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
            moved=(Q-Q.mean(0))@rot.T
        else: moved=(Q-Q.mean(0))@R.T+P.mean(0)*(a-.66)/.34
    fig,ax=plt.subplots(figsize=(10,6),facecolor="#F4F6FB")
    ax.scatter(*P.T,s=160,facecolors="none",edgecolors="#059669",linewidths=3,label="P target")
    ax.scatter(*moved.T,s=60,color="#4F46E5",label="Q transformed")
    ax.plot(*np.vstack([moved,moved[0]]).T,color="#4F46E5")
    ax.set(xlim=(-1.5,3),ylim=(-1,4.5),xlabel="x",ylabel="y",title="Known correspondence: center, rotate, translate")
    ax.set_aspect("equal");ax.legend(loc="upper left");ax.grid(alpha=.2)
    return fig
def main():
    parser=argparse.ArgumentParser();parser.add_argument("mode",choices=["chain","icp","calib","stereo"]);parser.add_argument("--frames",action="store_true");args=parser.parse_args()
    if args.mode=="icp":
        icp().savefig(OUT/"icp.png",dpi=130);plt.close("all")
        if args.frames:
            d=OUT/"icp-frames";d.mkdir(exist_ok=True)
            for i in range(48):icp(i).savefig(d/f"frame_{i:03}.png",dpi=100);plt.close("all")
    elif args.mode=="chain":
        fig=plt.figure(figsize=(10,6),facecolor="#F4F6FB");ax=fig.add_subplot(111,projection="3d")
        p=apply_T(T_BASE_CAM,[.2,0,2]);origin=T_BASE_CAM[:3,3]
        for anchor,rot,name in [(np.zeros(3),np.eye(3),"base"),(origin,R_BC,"camera"),(p,np.eye(3),"object")]:
            for j,col in enumerate(["#e11d48","#059669","#0284c7"]):ax.quiver(*anchor,*(rot[:,j]*.4),color=col)
            ax.text(*anchor,name)
        ax.scatter(*p,color="#4F46E5",s=80);ax.set(xlabel="x (m)",ylabel="y (m)",zlabel="z (m)",xlim=(-.5,2.8),ylim=(-1.65,1.65),zlim=(-1.25,2.05));ax.set_box_aspect([1,1,1]);fig.savefig(OUT/"chain.png",dpi=130)
    elif args.mode=="calib":
        poses,measured,delta,before,after=example();pred=np.array([fk(q) for q in poses]);corrected=np.array([fk(q+delta) for q in poses])
        fig,axs=plt.subplots(1,2,figsize=(12,5),facecolor="#F4F6FB")
        for ax,points,res,title in zip(axs,[pred,corrected],[before,after],["Before: RMS 11.6300 mm","After: RMS 0.0413 mm"]):
            ax.scatter(*points.T,color="#4F46E5",label="Prediction");ax.scatter(*measured.T,color="#059669",label="Synthetic measurement")
            ax.quiver(points[:,0],points[:,1],res[:,0],res[:,1],angles="xy",scale_units="xy",scale=1,color="#e11d48")
            ax.set_aspect("equal");ax.set(xlim=(.47,.71),ylim=(.48,.77),title=title,xlabel="x (m)",ylabel="y (m)");ax.grid(alpha=.2)
        axs[0].legend(fontsize=10);fig.tight_layout();fig.savefig(OUT/"calib.png",dpi=130)
    else:
        fig,axs=plt.subplots(1,2,figsize=(12,5),facecolor="#F4F6FB");d=np.linspace(2,50,300)
        axs[0].plot(d,50/d,color="#4F46E5");axs[0].set(xlabel="Disparity (px)",ylabel="Depth (m)",title="Z = 50 / disparity");axs[0].set_aspect("equal",adjustable="box")
        for x in [0,.1]:axs[1].plot([x,.2],[0,2],color="#4F46E5");axs[1].scatter([x],[0],color="#059669")
        axs[1].scatter([.2],[2],color="#e11d48");axs[1].set_aspect("equal");axs[1].set(xlabel="X (m)",ylabel="Z (m)",title="Two optical centers, one point");fig.tight_layout();fig.savefig(OUT/"stereo.png",dpi=130)
    print("Saved",args.mode,"in",OUT)
if __name__=="__main__":main()
