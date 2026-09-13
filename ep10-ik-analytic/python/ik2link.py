import argparse, math
from pathlib import Path

def wrap(a): return math.atan2(math.sin(a),math.cos(a))

def ik2(x,y,a1,a2,elbow=1,eps=1e-9):
    c2=(x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)
    if abs(c2)>1+eps: return None
    c2=max(-1.0,min(1.0,c2))
    s2=elbow*math.sqrt(max(0.0,1-c2*c2))
    t2=math.atan2(s2,c2)
    t1=math.atan2(y,x)-math.atan2(a2*s2,a1+a2*c2)
    return wrap(t1),wrap(t2)

def fk2(t1,t2,a1,a2):
    return a1*math.cos(t1)+a2*math.cos(t1+t2),a1*math.sin(t1)+a2*math.sin(t1+t2)

def solve(x,y,a1,a2):
    if not all(math.isfinite(v) for v in (x,y,a1,a2)) or min(a1,a2)<=0:
        raise ValueError("positive lengths and finite inputs required")
    if a1==a2 and x==0 and y==0: return "CONTINUUM"
    out=[]
    for e in (1,-1):
        q=ik2(x,y,a1,a2,e)
        if q is None: return "UNREACHABLE"
        fx,fy=fk2(*q,a1,a2)
        if math.hypot(fx-x,fy-y)>=1e-9: return "FK_TOLERANCE_EXCEEDED"
        if not any(math.hypot(wrap(q[0]-p[1]),wrap(q[1]-p[2]))<1e-9 for p in out):
            out.append((e,*q))
    return out

def pick(sols,cur,lo,hi):
    if isinstance(sols,str): return None
    ok=[s for s in sols if lo<=s[2]<=hi]
    return min(ok,key=lambda s:math.hypot(wrap(s[1]-cur[0]),wrap(s[2]-cur[1]))) if ok else None

def main():
    p=argparse.ArgumentParser()
    for k,v in [('a1',2.),('a2',1.5),('x',2.5),('y',1.5)]: p.add_argument('--'+k,type=float,default=v)
    p.add_argument('--elbow',choices=['both','down','up'],default='both')
    p.add_argument('--unit',choices=['rad','deg'],default='rad')
    p.add_argument('--out',default='solutions.csv')
    p.add_argument('--boundary',action='store_true')
    a=p.parse_args()
    if a.boundary: a.x=(a.a1+a.a2)*math.cos(math.pi/3);a.y=(a.a1+a.a2)*math.sin(math.pi/3)
    sols=solve(a.x,a.y,a.a1,a.a2)
    # 경계에서 합쳐진 해도 요청한 부호 갈래로 출력한다.
    if isinstance(sols,list) and len(sols)==1 and a.elbow=='up':
        sols=[(-1,*ik2(a.x,a.y,a.a1,a.a2,-1))]
    lines=['elbow,theta1,theta2,unit,fk_x,fk_y,status']
    if isinstance(sols,str): lines.append(f'none,,,{a.unit},,,{sols}')
    else:
        for e,t1,t2 in sols:
            name='down' if e==1 else 'up'
            if a.elbow not in ('both',name): continue
            fx,fy=fk2(t1,t2,a.a1,a.a2)
            u,v=(math.degrees(t1),math.degrees(t2)) if a.unit=='deg' else (t1,t2)
            z=[0.0 if abs(v)<.00005 else v for v in (u,v,fx,fy)]
            lines.append(f'{name},{z[0]:.4f},{z[1]:.4f},{a.unit},{z[2]:.4f},{z[3]:.4f},OK')
    text='\n'.join(lines)+'\n'
    Path(a.out).write_text(text,encoding='utf-8');print(text,end='')
if __name__=='__main__': main()
