"""EP.29: 문제 #119 운동계획과 #226 좌표 변환. 계산 함수는 목록을 반환한다."""
import argparse
import csv
import math
from pathlib import Path

POINTS = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)]


def trapezoidal_profile(distance=10., max_speed=2., acceleration=1., dt=.25):
    if min(distance, max_speed, acceleration, dt) <= 0:
        raise ValueError('거리, 최고속도, 가속도 크기, 시간 간격은 양수여야 합니다.')
    peak = min(max_speed, math.sqrt(distance * acceleration))
    ta = peak / acceleration
    da = .5 * acceleration * ta * ta
    cruise = max(0., (distance - 2 * da) / peak)
    t2, total = ta + cruise, 2 * ta + cruise
    # 종료점을 따로 붙여 나누어떨어지지 않는 간격에서도 정지 기록을 남긴다.
    times = [i * dt for i in range(math.ceil(total / dt)) if i * dt < total - 1e-12]
    times.append(total)
    rows = []
    for t in times:
        if t < ta:
            x, v, a = .5 * acceleration * t * t, acceleration * t, acceleration
        elif t < t2:
            x, v, a = da + peak * (t - ta), peak, 0.
        else:
            remaining = total - t
            x, v, a = distance - .5 * acceleration * remaining**2, acceleration * remaining, -acceleration
        rows.append((t, x, v, a))
    return rows


def transform_points(points, degrees=30., tx=1., ty=.5):
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return [(c*x-s*y+tx, s*x+c*y+ty) for x, y in points]


def write_outputs(directory, distance=10., dt=.25):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    rows = trapezoidal_profile(distance=distance, dt=dt)
    moved = transform_points(POINTS)
    with (directory/'trajectory.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'v', 'a'])
        writer.writerows([[f'{v:.10f}' for v in row] for row in rows])
    with (directory/'points.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['id', 'x', 'y', 'xp', 'yp'])
        for name, original, result in zip('ABCD', POINTS, moved):
            writer.writerow([name]+[f'{v:.10f}' for v in (*original, *result)])
    print(f'rows={len(rows)} T={rows[-1][0]:.10f} x={rows[-1][1]:.10f} v={rows[-1][2]:.10f} a={rows[-1][3]:.10f}')
    for name, (x, y) in zip('ABCD', moved):
        print(f'{name} {x:.10f} {y:.10f}')
    return rows, moved


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='.')
    parser.add_argument('--distance', type=float, default=10.)
    parser.add_argument('--dt', type=float, default=.25)
    args = parser.parse_args()
    write_outputs(args.out, args.distance, args.dt)
