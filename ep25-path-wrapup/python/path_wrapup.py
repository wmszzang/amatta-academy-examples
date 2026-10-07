"""EP.25: 동일 입력으로 경로·비용·실패 상태를 검증한다."""
import heapq
import math
from collections import deque

COST_GRID = [[1, 3, 1], [1, 5, 1], [4, 2, 1]]
MAP = [[0, 0, 0, 0], [1, 1, 0, 1], [0, 0, 0, 0], [0, 1, 1, 0]]
SAMPLES = [(4., 0.), (0., 2.), (2., 2.), (4., 2.), (4., 0.), (4., -1.), (2., 1.2)]
DIRS = [(-1, 0), (0, -1), (0, 1), (1, 0)]

def restore(parent, goal):
    path = [goal]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    return path[::-1]

def search(grid, start, goal, weighted=False):
    rows, cols = len(grid), len(grid[0])
    valid = lambda p: 0 <= p[0] < rows and 0 <= p[1] < cols and (weighted or grid[p[0]][p[1]] == 0)
    if not valid(start) or not valid(goal):
        return [], math.inf, []
    heuristic = lambda p: 0 if weighted else abs(p[0]-goal[0])+abs(p[1]-goal[1])
    initial = grid[start[0]][start[1]] if weighted else 0
    best, parent, popped = {start: initial}, {}, []
    queue = [(initial+heuristic(start), initial, *start)]
    while queue:
        _, cost, row, col = heapq.heappop(queue)
        here = (row, col)
        if cost != best[here]:
            continue
        popped.append((here, cost))
        if here == goal:
            return restore(parent, goal), cost, popped
        for dr, dc in DIRS:
            nxt = row+dr, col+dc
            if not valid(nxt):
                continue
            candidate = cost+(grid[nxt[0]][nxt[1]] if weighted else 1)
            if candidate < best.get(nxt, math.inf):
                best[nxt], parent[nxt] = candidate, here
                heapq.heappush(queue, (candidate+heuristic(nxt), candidate, *nxt))
    return [], math.inf, popped

def segment_distance(a, b, point):
    ux, uy = b[0]-a[0], b[1]-a[1]
    denominator = ux*ux+uy*uy
    t0 = ((point[0]-a[0])*ux+(point[1]-a[1])*uy)/denominator if denominator else 0
    t = min(1, max(0, t0))
    closest = a[0]+t*ux, a[1]+t*uy
    return math.dist(point, closest), t0, t, closest

def safe(a, b):
    return segment_distance(a, b, (2, 0))[0] > 1

def sampling(samples=SAMPLES, improve=True):
    nodes = [{'point': (0., 0.), 'parent': -1, 'cost': 0., 'name': 'S'}]
    events = []
    def ancestor(index, target):
        while index >= 0:
            if index == target:
                return True
            index = nodes[index]['parent']
        return False
    def propagate(index):
        for child, node in enumerate(nodes):
            if node['parent'] == index:
                node['cost'] = nodes[index]['cost']+math.dist(nodes[index]['point'], node['point'])
                events.append(('propagate', child, index, node['cost']))
                propagate(child)
    for sample in samples:
        nearest = min(range(len(nodes)), key=lambda i: math.dist(nodes[i]['point'], sample))
        origin = nodes[nearest]['point']
        distance = math.dist(origin, sample)
        if distance == 0:
            events.append(('duplicate', -1, -1, 0.))
            continue
        ratio = min(1., 3./distance)
        point = origin[0]+ratio*(sample[0]-origin[0]), origin[1]+ratio*(sample[1]-origin[1])
        if not (-1 <= point[0] <= 5 and -1 <= point[1] <= 3) or not safe(origin, point):
            events.append(('reject', -1, -1, 0.))
            continue
        if any(math.dist(n['point'], point) < 1e-12 for n in nodes):
            events.append(('duplicate', -1, -1, 0.))
            continue
        neighbors = [i for i, n in enumerate(nodes) if math.dist(n['point'], point) <= 3.]
        parent = nearest
        if improve:
            parent = min((i for i in neighbors if safe(nodes[i]['point'], point)), key=lambda i: nodes[i]['cost']+math.dist(nodes[i]['point'], point))
        index = len(nodes)
        cost = nodes[parent]['cost']+math.dist(nodes[parent]['point'], point)
        nodes.append({'point': point, 'parent': parent, 'cost': cost, 'name': ['S','A','B','D','G','E','N'][index] if index < 7 else str(index)})
        events.append(('insert', index, parent, cost))
        if improve:
            for neighbor in neighbors:
                node = nodes[neighbor]
                candidate = cost+math.dist(point, node['point'])
                if candidate < node['cost'] and not ancestor(index, neighbor) and safe(point, node['point']):
                    node['parent'], node['cost'] = index, candidate
                    events.append(('rewire', neighbor, index, candidate))
                    propagate(neighbor)
    goal = next((i for i,n in enumerate(nodes) if math.dist(n['point'], (4,0)) < 1e-12), None)
    path = []
    cursor = goal
    while cursor is not None and cursor >= 0:
        path.append(cursor)
        cursor = nodes[cursor]['parent']
    return nodes, path[::-1], events

def evaluate(commands=((.5,0),(.5,.3),(.3,0)), distances=(2.,1.5,2.5)):
    if len(commands) != len(distances):
        raise ValueError('후보와 거리 개수가 다릅니다')
    rows = []
    for (v,w), distance in zip(commands,distances):
        x, y, angle = v*.1, 0., w*.1
        error = math.atan2(math.sin(math.atan2(-y,2-x)-angle), math.cos(math.atan2(-y,2-x)-angle))
        h, velocity, clearance = 1/(1+abs(error)), v, min(distance/3,1)
        rows.append((v,w,x,y,angle,h,velocity,clearance,.4*h+.2*velocity+.4*clearance))
    return rows, max(rows,key=lambda row: row[-1]) if rows else None

def bfs(grid, start, goal):
    queue, seen, parent = deque([start]), {start}, {}
    while queue:
        here = queue.popleft()
        if here == goal:
            return restore(parent,goal)
        for dr,dc in DIRS:
            point = here[0]+dr,here[1]+dc
            if 0 <= point[0] < len(grid) and 0 <= point[1] < len(grid[0]) and grid[point[0]][point[1]] == 0 and point not in seen:
                seen.add(point)
                parent[point] = here
                queue.append(point)
    return []

def coverage(grid):
    order = [(row,col) for row in range(len(grid)) for col in (range(len(grid[0])) if row%2 == 0 else reversed(range(len(grid[0])))) if grid[row][col] == 0]
    if not order:
        return [], [], []
    route, unreachable = [order[0]], []
    for target in order[1:]:
        part = bfs(grid,route[-1],target)
        if part:
            route.extend(part[1:])
        else:
            unreachable.append(target)
    return order, route, unreachable

def verify():
    path,cost,pops = search(COST_GRID,(0,0),(2,2),True)
    assert cost == 7 and sum(COST_GRID[r][c] for r,c in path) == cost
    assert [p for p,c in pops] == [(0,0),(1,0),(0,1),(0,2),(1,2),(2,0),(1,1),(2,2)]
    path,cost,_ = search(MAP,(0,0),(3,3))
    assert cost == 6 and len(path) == 7
    assert all(abs(a[0]-b[0])+abs(a[1]-b[1]) == 1 for a,b in zip(path,path[1:]))
    blocked = [row[:] for row in MAP]
    blocked[1][2] = 1
    assert not search(blocked,(0,0),(3,3))[0]
    assert search([[0]],(0,0),(0,0))[1] == 0
    assert segment_distance((0,0),(0,0),(3,4))[0] == 5
    assert segment_distance((0,0),(4,0),(2,1))[0] == 1
    assert segment_distance((0,0),(4,0),(6,0))[0] == 2
    nodes,path,_ = sampling()
    assert path == [0,6,4]
    assert abs(nodes[4]['cost']-4.66476151587624) < 1e-9
    assert abs(nodes[5]['cost']-5.66476151587624) < 1e-9
    assert all(safe(nodes[a]['point'],nodes[b]['point']) for a,b in zip(path,path[1:]))
    assert abs(sum(math.dist(nodes[a]['point'],nodes[b]['point']) for a,b in zip(path,path[1:]))-nodes[path[-1]]['cost']) < 1e-12
    assert not sampling(SAMPLES[:1])[1]
    assert len(sampling(SAMPLES+[SAMPLES[-1]])[0]) == 7
    assert sampling(SAMPLES[:5],False)[0][4]['cost'] == 8
    assert evaluate()[1][:2] == (.3,0)
    assert evaluate([],[])[1] is None
    assert coverage([[1]]) == ([],[],[])
    assert coverage([[0,1,0]])[2] == [(0,2)]
    assert coverage([[0,0,0],[0,1,0]])[1] == [(0,0),(0,1),(0,2),(1,2),(0,2),(0,1),(0,0),(1,0)]

def output():
    for label,grid,goal,weighted in [('DIJKSTRA',COST_GRID,(2,2),True),('ASTAR',MAP,(3,3),False)]:
        path,cost,pops = search(grid,(0,0),goal,weighted)
        print(f'{label},cost,{cost:.9f}')
        for point,c in pops:
            print(f'{label},pop,{point[0]},{point[1]},{c:.9f}')
        for r,c in path:
            print(f'{label},path,{r},{c}')
    nodes,path,events = sampling()
    for kind,i,p,cost in events:
        print(f'RRTSTAR,{kind},{i},{p},{cost:.9f}')
    for i,n in enumerate(nodes):
        print(f'RRTSTAR,node,{i},{n["point"][0]:.9f},{n["point"][1]:.9f},{n["parent"]},{n["cost"]:.9f}')
    print('RRTSTAR,path,'+','.join(map(str,path)))
    print(f'RRT,cost,{sampling(SAMPLES[:5],False)[0][4]["cost"]:.9f}')
    for row in evaluate()[0]:
        print('DWA,'+','.join(f'{x:.9f}' for x in row))
    print('DWA,best,0.300000000,0.000000000')
    for i,grid in enumerate([[[0,0,0],[0,1,0]],[[0,1],[0,0],[1,0]],[[0,1,0]]]):
        for label,points in zip(['order','route','unreachable'],coverage(grid)):
            for r,c in points:
                print(f'COVERAGE,{i},{label},{r},{c}')
    print('BOUNDARIES,PASS')

if __name__ == '__main__':
    verify()
    output()
