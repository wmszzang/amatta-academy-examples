import math
import numpy as np

def estimate_svd(q,p):
    q,p=np.asarray(q,dtype=float),np.asarray(p,dtype=float)
    if q.shape!=p.shape or q.ndim!=2 or q.shape[1]!=2 or len(q)<2: raise ValueError('INVALID_POINTS')
    cq,cp=q.mean(axis=0),p.mean(axis=0)
    h=(q-cq).T@(p-cp)
    u,s,vt=np.linalg.svd(h)
    if s[0]<1e-12: raise ValueError('DEGENERATE_POINTS')
    v=vt.T; rotation=v@u.T
    reflected=np.linalg.det(rotation)<0
    if reflected:
        v[:,-1]*=-1
        rotation=v@u.T
    translation=cp-rotation@cq
    return math.atan2(rotation[1,0],rotation[0,0]),tuple(translation),rotation,reflected

def estimate_closed(q,p):
    cq=[sum(z[i] for z in q)/len(q) for i in range(2)]
    cp=[sum(z[i] for z in p)/len(p) for i in range(2)]
    num=den=0.0
    for a,b in zip(q,p):
        x,y=a[0]-cq[0],a[1]-cq[1]; u,v=b[0]-cp[0],b[1]-cp[1]
        num+=x*v-y*u; den+=x*u+y*v
    angle=math.atan2(num,den); c,s=math.cos(angle),math.sin(angle)
    return angle,(cp[0]-c*cq[0]+s*cq[1],cp[1]-s*cq[0]-c*cq[1])
