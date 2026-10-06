"""실제 계산값으로 정적 그림과 12fps 재생용 프레임을 만든다."""
from pathlib import Path
import argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from sampling_planning import SAMPLES, rrt, ledger, score_candidates, coverage, travel


def draw(step=5, output='sampling_planning.png'):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), facecolor='#F4F6FB')
    for ax in axes.flat:
        ax.set_facecolor('#F4F6FB')
    ax = axes[0, 0]
    nodes, parents, events, path = rrt(SAMPLES[:step])
    ax.add_patch(Circle((2, 0), 1, color='#FDF6E3', ec='#E11D48'))
    for i in range(1, len(nodes)):
        a, b = nodes[parents[i]], nodes[i]
        ax.plot([a[0], b[0]], [a[1], b[1]], '-o', color='#4F46E5')
    ax.scatter([0, 4], [0, 0], color='#059669')
    ax.set(xlim=(-1, 5), ylim=(-1.2, 3), title=f'RRT: fixed samples, step {step}/5', xlabel='x [m]', ylabel='y [m]')
    ax.set_aspect('equal')
    ax.grid(alpha=.2)
    ax = axes[0, 1]
    costs, _ = ledger()
    ax.bar(['new node', 'neighbor', 'descendant'], [6, 10, 11], color='#C7D2FE', label='before')
    ax.bar(['new node', 'neighbor', 'descendant'], [costs[5], costs[3], costs[4]], width=.45, color='#4F46E5', label='after')
    ax.set(title='RRT*: independent cost ledger', ylabel='Accumulated cost')
    ax.legend()
    ax = axes[1, 0]
    rows, best = score_candidates([(.5, 0), (.5, .3), (.3, 0)], [2, 1.5, 2.5])
    bars = ax.bar(['(0.5,0.0)', '(0.5,0.3)', '(0.3,0.0)'], [r[-1] for r in rows], color=['#4F46E5', '#4F46E5', '#059669'])
    ax.bar_label(bars, labels=[f'{r[-1]:.6f}' for r in rows], padding=3)
    ax.set(ylim=(0, 1), title='#401: score only, not a safety verdict', ylabel='G')
    ax = axes[1, 1]
    grid = [[0, 0, 0], [0, 1, 0]]
    visits, (movement, unreachable) = coverage(grid), travel(grid)
    for r, row in enumerate(grid):
        for c, blocked in enumerate(row):
            ax.add_patch(Rectangle((c-.45, r-.45), .9, .9, fc='#64748B' if blocked else '#EEF2FF', ec='#1E293B'))
            if not blocked:
                ax.text(c, r, str(visits.index((r, c))+1), ha='center', va='center')
    # 기록의 재방문을 숨기지 않도록 연결선과 목록을 따로 표기한다.
    ax.plot([c for r,c in movement], [r for r,c in movement], color='#059669', linewidth=3, alpha=.6)
    ax.set(xlim=(-.7, 2.7), ylim=(1.7, -.7), xticks=range(3), yticks=range(2), xlabel='column', ylabel='row', title='#505: visit order (numbers), travel (line)')
    ax.set_aspect('equal')
    fig.tight_layout(pad=2)
    fig.savefig(output, dpi=120)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', action='store_true')
    args = parser.parse_args()
    draw()
    if args.frames:
        out = Path('frames')
        out.mkdir(exist_ok=True)
        for i in range(60):
            draw(min(5, i//12+1), out / f'frame-{i:03d}.png')
    print('Created sampling_planning.png' + (' and 60 frames (12fps)' if args.frames else ''))
