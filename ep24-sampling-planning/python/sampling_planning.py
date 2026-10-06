"""EP.24 교육용 재현 예제. 실제 로봇 주행 명령을 보내지 않는다."""
import math
from collections import deque

SAMPLES = [(4., 0.), (0., 2.), (2., 2.), (4., 2.), (4., 0.)]


def segment_distance(a, b, center):
    dx, dy = b[0]-a[0], b[1]-a[1]
    norm = dx*dx+dy*dy
    t = 0.0 if norm == 0 else max(0., min(1., ((center[0]-a[0])*dx+(center[1]-a[1])*dy)/norm))
    return math.hypot(a[0]+t*dx-center[0], a[1]+t*dy-center[1])


def rrt(samples, limit=None):
    nodes, parents, events = [(0., 0.)], [-1], []
    for sample in samples[:limit]:
        near = min(range(len(nodes)), key=lambda i: math.dist(nodes[i], sample))
        a = nodes[near]
        distance = math.dist(a, sample)
        if distance == 0:
            events.append((sample, a, a, segment_distance(a, a, (2., 0.)), 'DUPLICATE'))
            continue
        fraction = min(1., 2./distance)
        b = (a[0]+fraction*(sample[0]-a[0]), a[1]+fraction*(sample[1]-a[1]))
        clearance = segment_distance(a, b, (2., 0.))
        status = 'ACCEPT' if clearance > 1. and -1 <= b[0] <= 5 and -1 <= b[1] <= 3 else 'REJECT'
        events.append((sample, a, b, clearance, status))
        if status == 'ACCEPT':
            nodes.append(b)
            parents.append(near)
            if math.dist(b, (4., 0.)) < 1e-9:
                path, cursor = [], len(nodes)-1
                while cursor >= 0:
                    path.append(nodes[cursor])
                    cursor = parents[cursor]
                return nodes, parents, events, path[::-1]
    return nodes, parents, events, []


def rewire(parent, costs, edge_costs, node, new_parent, link_cost):
    # 조상에 연결하면 순환이 생기므로 비용 장부를 바꾸기 전에 거부한다.
    cursor = new_parent
    while cursor >= 0:
        if cursor == node:
            raise ValueError('cycle')
        cursor = parent[cursor]
    parent[node], edge_costs[node] = new_parent, link_cost
    pending = [node]
    while pending:
        current = pending.pop()
        costs[current] = costs[parent[current]]+edge_costs[current]
        pending.extend(i for i, p in enumerate(parent) if p == current)


def ledger():
    parent, costs, edge = [-1, 0, 0, 1, 3, 2], [0., 7., 4., 10., 11., 6.], [0., 7., 4., 3., 1., 2.]
    chosen = min([(1, 1.), (2, 2.)], key=lambda pair: costs[pair[0]]+pair[1])
    parent[5], edge[5] = chosen
    costs[5] = costs[chosen[0]]+chosen[1]
    rewire(parent, costs, edge, 3, 5, 1.)
    return costs, parent


def score_candidates(candidates, distances):
    rows = []
    for (v, w), distance in zip(candidates, distances):
        x, y, theta = v*.1, 0., w*.1
        delta = math.atan2(0.-y, 2.-x)-theta
        delta = math.atan2(math.sin(delta), math.cos(delta))
        h, velocity, clearance = 1./(1.+abs(delta)), v/1., min(distance/3., 1.)
        score = .4*h+.2*velocity+.4*clearance
        rows.append((v,w,x,y,theta,h,velocity,clearance,score))
    return rows, max(range(len(rows)), key=lambda i: rows[i][-1]) if rows else None


def dynamic_window(v=.4, w=0., dt=.1):
    return (max(0.,v-2*dt),min(1.,v+2*dt),max(-1.,w-3*dt),min(1.,w+3*dt))


def safe_straight(candidates, free_distance, deceleration=1.):
    return [v for v in candidates if v*v/(2*deceleration) < free_distance]


def coverage(grid):
    return [(r,c) for r,row in enumerate(grid)
            for c in (range(len(row)) if r%2==0 else range(len(row)-1,-1,-1)) if row[c]==0]


def connect(grid, start, goal):
    queue, parent = deque([start]), {start: None}
    while queue:
        p = queue.popleft()
        if p == goal:
            path=[]
            while p is not None:
                path.append(p); p=parent[p]
            return path[::-1]
        for dr,dc in [(-1,0),(0,-1),(0,1),(1,0)]:
            q=(p[0]+dr,p[1]+dc)
            if 0<=q[0]<len(grid) and 0<=q[1]<len(grid[q[0]]) and grid[q[0]][q[1]]==0 and q not in parent:
                parent[q]=p; queue.append(q)
    return []


def travel(grid):
    visits=coverage(grid)
    if not visits:return [],[]
    path, unreachable=[visits[0]],[]
    for target in visits[1:]:
        part=connect(grid,path[-1],target)
        if part:path.extend(part[1:])
        else:unreachable.append(target)
    return path,unreachable


def run():
    nodes,parents,events,path=rrt(SAMPLES)
    print('RRT,step,sx,sy,ax,ay,bx,by,d,status')
    for i,(sample,a,b,d,status) in enumerate(events,1):
        print(f'RRT,{i},'+','.join(f'{x:.9f}' for x in (*sample,*a,*b,d))+','+status)
    for i,(x,y) in enumerate(path):print(f'PATH,{i},{x:.9f},{y:.9f}')
    print(f'LENGTH,{sum(math.dist(a,b) for a,b in zip(path,path[1:])):.9f}')
    print(f'TANGENT,{segment_distance((0,0),(4,0),(2,1)):.9f},COLLISION')
    costs, parent=ledger()
    print(f'REWIRE,{parent[5]},{costs[5]:.9f},{costs[3]:.9f},{costs[4]:.9f}')
    print('WINDOW,'+','.join(f'{x:.9f}' for x in dynamic_window()))
    print(f'BRAKE,{.6*.6/2:.9f},REJECT')
    rows,best=score_candidates([(.5,0.),(.5,.3),(.3,0.)],[2.,1.5,2.5])
    print('DWA,index,v,w,x,y,theta,H,V,C,G')
    for i,row in enumerate(rows):print(f'DWA,{i},'+','.join(f'{x:.9f}' for x in row))
    print(f'BEST,{best},{rows[best][0]:.9f},{rows[best][1]:.9f}')
    for i,grid in enumerate([[[0,0,0],[0,1,0]],[[0,1],[0,0],[1,0]]],1):
        print(f'VISITS{i},'+';'.join(f'{r}:{c}' for r,c in coverage(grid)))
        if i==1:print('TRAVEL1,'+';'.join(f'{r}:{c}' for r,c in travel(grid)[0]))
    assert path==[(0,0),(0,2),(2,2),(4,2),(4,0)]
    assert segment_distance((2,1),(2,1),(2,0))==1.
    assert rrt([(0,0)])[2][0][-1]=='DUPLICATE'
    assert rrt(SAMPLES,1)[3]==[]
    assert costs[3:5]==[7.,8.]
    assert best==2 and abs(rows[best][-1]-.7933333333333333)<1e-12
    assert not safe_straight([.6],.15)
    assert coverage([[1,1]])==[]
    assert travel([[0,1,0]])[1]==[(0,2)]
    assert len(travel([[0,0,0],[0,1,0]])[0])-1==7
    print('CHECKS,tangent,zero-length,duplicate,not-found,no-safe-speed,blocked-grid,disconnected,PASS')


if __name__=='__main__':
    run()
