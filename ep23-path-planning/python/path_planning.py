"""EP.23: 네 방향 격자 탐색과 점-선분 거리. 모든 좌표는 무단위다."""
import csv
import heapq
import math

TERRAIN = [[1, 3, 1], [1, 5, 1], [4, 2, 1]]
GRID = [[0, 0, 0, 0], [1, 1, 0, 1], [0, 0, 0, 0], [0, 1, 1, 0]]


def search(grid, start=(0, 0), goal=None, terrain=False, heuristic=True):
    rows, cols = len(grid), len(grid[0])
    goal = goal or (rows - 1, cols - 1)
    if not terrain and (grid[start[0]][start[1]] or grid[goal[0]][goal[1]]):
        return None, [], []
    def h(r, c):
        return abs(r - goal[0]) + abs(c - goal[1]) if heuristic and not terrain else 0
    initial = grid[start[0]][start[1]] if terrain else 0
    costs, parent = {start: initial}, {}
    queue = [(initial + h(*start), initial, *start)]
    trace = []
    while queue:
        f, g, r, c = heapq.heappop(queue)
        if g != costs[(r, c)]:
            continue
        trace.append((r, c, g, h(r, c), f))
        if (r, c) == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return g, path[::-1], trace
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if not terrain and grid[nr][nc]:
                continue
            new = g + (grid[nr][nc] if terrain else 1)
            if new < costs.get((nr, nc), math.inf):
                costs[(nr, nc)], parent[(nr, nc)] = new, (r, c)
                heapq.heappush(queue, (new + h(nr, nc), new, nr, nc))
    return None, [], trace


def point_segment(a, b, p):
    vx, vy = b[0] - a[0], b[1] - a[1]
    denominator = vx * vx + vy * vy
    # 퇴화한 선분은 나눗셈 전에 점으로 처리한다.
    raw = None if denominator == 0 else ((p[0]-a[0])*vx + (p[1]-a[1])*vy)/denominator
    t = 0.0 if raw is None else max(0.0, min(1.0, raw))
    q = a[0] + t*vx, a[1] + t*vy
    return math.hypot(p[0]-q[0], p[1]-q[1]), q, raw, t


def main():
    cases = [('P1', TERRAIN, True, False), ('P2', GRID, False, True), ('P2_h0', GRID, False, False)]
    blocked = [row[:] for row in GRID]
    blocked[1][2] = 1
    cases += [('NO_PATH', blocked, False, True), ('ONE_TERRAIN', [[7]], True, False), ('ONE_GRID', [[0]], False, True),
              ('P1_EXTRA1', [[1,2],[1,1]], True, False), ('P1_EXTRA2', [[1,9,9],[1,9,1],[1,1,1]], True, False)]
    with open('paths.csv','w',newline='',encoding='utf-8') as pf, open('trace.csv','w',newline='',encoding='utf-8') as tf:
        pw, tw = csv.writer(pf,lineterminator='\n'), csv.writer(tf,lineterminator='\n')
        pw.writerow(['case','step','row','col']); tw.writerow(['case','step','row','col','g','h','f'])
        for name, grid, terrain, heuristic in cases:
            cost, path, trace = search(grid, terrain=terrain, heuristic=heuristic)
            print(f'{name}: NO PATH' if cost is None else f'{name}: cost={cost} points={len(path)} moves={len(path)-1} popped={len(trace)}')
            for i,(r,c) in enumerate(path): pw.writerow([name,i,r,c])
            for i,row in enumerate(trace): tw.writerow([name,i,*row])
    with open('distances.csv','w',newline='',encoding='utf-8') as df:
        w = csv.writer(df,lineterminator='\n'); w.writerow(['case','t_raw','t','qx','qy','distance','result'])
        for name,b,p,radius in [('P3_IN',(4,0),(2,3),None),('P3_AFTER',(4,0),(6,0),None),('P3_BEFORE',(4,0),(-1,-1),None),('P3_ZERO',(0,0),(2,3),None),('P4_TOUCH',(4,0),(2,1),1),('P4_SAFE',(4,0),(2,2),1)]:
            d,q,raw,t = point_segment((0,0),b,p)
            result = '-' if radius is None else ('COLLISION' if d <= radius else 'SAFE')
            w.writerow([name,'NA' if raw is None else f'{raw:.4f}',f'{t:.4f}',f'{q[0]:.4f}',f'{q[1]:.4f}',f'{d:.4f}',result])
            print(f'{name}: d={d:.4f} {result}')


if __name__ == '__main__':
    main()
