"""EP.30 대표 입력 일곱 유형 검산. 외부 패키지 없이 실행한다."""
import csv
import math
from collections import deque


def calculate():
    rows = []
    def save(name, values, expected):
        if not all(math.isclose(a, b, rel_tol=0, abs_tol=1e-8) for a,b in zip(values,expected)):
            raise AssertionError(name)
        rows.append((name, *values, 'PASS'))
    distance, speed, accel = 10., 2., 1.
    ta = speed / accel
    tc = (distance - speed*speed/accel) / speed
    save('motion_119', [ta, tc, 2*ta+tc, speed*(ta+tc)], [2,3,7,10])
    c,s=math.cos(math.pi/6),math.sin(math.pi/6)
    for i,(x,y) in enumerate([(0,0),(1,0),(1,1),(0,1)]):
        values=[x*c-y*s+1, x*s+y*c+.5]
        expected=[(1,.5),(1.8660254037844386,1),(1.3660254037844386,1.8660254037844386),(.5,1.3660254037844386)][i]
        save(f'transform_226_{i}', values, expected)
    q1,q2=map(math.radians,[25,50])
    fk=[1.4*math.cos(q1)+.9*math.cos(q1+q2),1.4*math.sin(q1)+.9*math.sin(q1+q2)]
    save('fk_ep09',fk,[1.5017680424435782,1.4609988100971407])
    c2=(1.5**2+1**2-2)/2
    s2=math.sqrt(1-c2*c2)
    b=math.atan2(s2,c2)
    a=math.atan2(1,1.5)-math.atan2(s2,1+c2)
    save('ik_222',[math.degrees(a),math.degrees(b),math.cos(a)+math.cos(a+b),math.sin(a)+math.sin(a+b)],
         [8.031161252724512,51.31781254651057,1.5,1])
    j=[[-2*math.sin(.5)-1.5*math.sin(1),-1.5*math.sin(1)],
       [2*math.cos(.5)+1.5*math.cos(1),1.5*math.cos(1)]]
    vel=[row[0]*.4+row[1]*-.3 for row in j]
    save('jacobian_239',vel,[-.509761078604546,.7831113953925192])
    grid=[[0,0,0,0],[1,1,0,1],[0,0,0,0],[0,1,1,0]]
    queue=deque([(0,0)])
    previous={(0,0):None}
    while queue:
        current=queue.popleft()
        if current==(3,3):break
        for dr,dc in [(0,1),(1,0),(0,-1),(-1,0)]:
            nxt=(current[0]+dr,current[1]+dc)
            if 0<=nxt[0]<4 and 0<=nxt[1]<4 and not grid[nxt[0]][nxt[1]] and nxt not in previous:
                previous[nxt]=current
                queue.append(nxt)
    route=[]
    current=(3,3)
    while current is not None:
        route.append(current)
        current=previous[current]
    route.reverse()
    assert all(abs(a[0]-b[0])+abs(a[1]-b[1])==1 for a,b in zip(route,route[1:]))
    save('path_241',[len(route),len(route)-1,6],[7,6,6])
    x,p=0.,1.
    for i,z in enumerate([1.2,2.1]):
        xp,pp=x+1,p+.1
        k=pp/(pp+.5)
        x,p=xp+k*(z-xp),(1-k)*pp
        expected=[(1.1375,.34375),(2.119867549668874,.23509933774834438)][i]
        save(f'kalman_399_{i+1}',[x,p],expected)
    # 짧은 거리·경계·각도 래핑도 같은 기존 입력으로 점검한다.
    save('triangle_ep29',[math.sqrt(1*1),2*math.sqrt(1/1)],[1,2])
    save('boundary_ep29',[(4-2*2/1)/2,4],[0,4])
    save('endpoint_119',[10-.5*(7-6.75)**2,7-6.75],[9.96875,.25])
    save('wrap_ep13',[(3-(-3)+math.pi)%(2*math.pi)-math.pi],[-.28318530717958623])
    return rows


def main():
    rows=calculate()
    with open('crash_check.csv','w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f,lineterminator='\n')
        for row in rows:
            values=[row[0],*[f'{v:.10f}' for v in row[1:-1]],row[-1]]
            writer.writerow(values)
            print(','.join(values))


if __name__=='__main__':
    main()
