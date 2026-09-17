"""EP.12: 도달 판정과 요구조건 판정은 서로 다른 출력이다."""
import argparse
import math
from pathlib import Path
from kin_common import ik2, fk, wrap, jacobian
from hull_area import convex_hull, shoelace
from cspace import cspace_grid

TARGETS = [(2.5,1.5),(1,2.6),(3.4,.5),(.3,.2),(3.6,.2),(-1,-2)]
LIMITS = ((-30,120),(15,150))

def manip(t1, t2, a1=2, a2=1.5):
    return a1*a2*abs(math.sin(t2))

def cond(t1, t2, a1=2, a2=1.5):
    j = jacobian(a1,a2,t1,t2)
    e = sum(v*v for row in j for v in row)
    d = abs(j[0][0]*j[1][1]-j[0][1]*j[1][0])
    delta = math.sqrt(max(e*e-4*d*d,0))
    high, low = math.sqrt((e+delta)/2), math.sqrt(max((e-delta)/2,0))
    return high/low if low > 1e-12 else math.inf

def allowed(q, limits=LIMITS):
    return all(lo-1e-10 <= math.degrees(wrap(t)) <= hi+1e-10
               for t,(lo,hi) in zip(q,limits))

def spec_check(x,y,a1=2,a2=1.5,lim1=LIMITS[0],lim2=LIMITS[1],w_min=1,k_max=10,tol=1e-6):
    r = math.hypot(x,y)
    if r < abs(a1-a2)-1e-12: return ('FAIL_UNREACHABLE_INNER',None)
    if r > a1+a2+1e-12: return ('FAIL_UNREACHABLE_OUTER',None)
    choices = [q for el in (1,-1) if (q:=ik2(x,y,a1,a2,el)) is not None and allowed(q,(lim1,lim2))]
    if not choices: return ('FAIL_JOINT_LIMIT',None)
    q = max(choices,key=lambda q:manip(*q,a1,a2))
    w,k = manip(*q,a1,a2),cond(*q,a1,a2)
    fx,fy = fk(*q,a1,a2)
    error = math.hypot(fx-x,fy-y)
    result = (*map(math.degrees,q),w,k,error)
    if error > tol: return ('FAIL_PRECISION',result)
    if w < w_min: return ('FAIL_LOW_MANIPULABILITY',result)
    return ('PASS' if k <= k_max else 'FAIL_ILL_CONDITIONED',result)

def sweep(a1=2,a2=1.5,step=5,limits=LIMITS):
    for i in range(round((limits[0][1]-limits[0][0])/step)+1):
        for j in range(round((limits[1][1]-limits[1][0])/step)+1):
            q = (math.radians(limits[0][0]+i*step),math.radians(limits[1][0]+j*step))
            yield (*fk(*q,a1,a2),manip(*q,a1,a2),cond(*q,a1,a2))

def grid_area(a1=2,a2=1.5,limits=LIMITS,h=.005):
    # 각 셀 중심을 검사한다. 표본화 방식에 따라 마지막 자리 값이 달라진다.
    reach = a1+a2
    n = math.ceil(2*reach/h)
    hits = 0
    for i in range(n):
        x = -reach+(i+.5)*h
        for j in range(n):
            y = -reach+(j+.5)*h
            if not abs(a1-a2) <= math.hypot(x,y) <= reach: continue
            if any(q is not None and allowed(q,limits) for q in (ik2(x,y,a1,a2,1),ik2(x,y,a1,a2,-1))):
                hits += 1
    return hits*h*h

def write_outputs(out='.'):
    out = Path(out);out.mkdir(parents=True,exist_ok=True)
    points = list(sweep())
    hull = convex_hull([p[:2] for p in points])
    area = grid_area()
    def save(name,header,rows):
        def fmt(v):
            if isinstance(v,str): return v
            return f'{0.0 if abs(v)<.0000005 else v:.6f}'
        (out/name).write_text(header+'\n'+''.join(','.join(fmt(v) for v in row)+'\n' for row in rows),encoding='utf-8')
    save('workspace.csv','x,y,w,kappa',points)
    save('hull.csv','x,y',hull)
    rows=[]
    for x,y in TARGETS:
        verdict,result=spec_check(x,y)
        rows.append((x,y,verdict,*(result if result else ('','','','',''))))
        print(f'({x}, {y}) {verdict} '+(' '.join(f'{v:.4f}' for v in result[:4]) if result else ''))
    save('spec.csv','x,y,verdict,th1,th2,w,kappa,fk_err',rows)
    free,hit=cspace_grid()
    rows=sorted([(a,b,'free') for a,b in free]+[(a,b,'obstacle') for a,b in hit])
    save('cspace.csv','th1,th2,state',rows)
    (out/'area.txt').write_text(f'hull_area={shoelace(hull):.6f}\ngrid_area={area:.6f}\nratio={shoelace(hull)/area:.6f}\n',encoding='utf-8')
    print(f'samples={len(points)} hull_vertices={len(hull)} hull_area={shoelace(hull):.7f} grid_area={area:.7f}')
    print(f'C-obstacle={len(hit)} C-free={len(free)}')
    print(f'dexterous_area={math.pi*(.1**2+1.5**2-.5**2):.4f}')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',default='.')
    write_outputs(p.parse_args().out)
