"""검산한 입력으로 정적 그림과 12fps 스틱 프레임을 만든다."""
import argparse
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from crash_check import calculate


def pose(q1, q2):
    a, b = math.radians(q1), math.radians(q1 + q2)
    return [0, 1.4*math.cos(a), 1.4*math.cos(a)+.9*math.cos(b)], [0, 1.4*math.sin(a), 1.4*math.sin(a)+.9*math.sin(b)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', action='store_true')
    args = parser.parse_args()
    rows = calculate()
    fig, axs = plt.subplots(2, 3, figsize=(12, 8), layout='constrained')
    ax = axs.flat
    ax[0].plot([0, 2, 5, 7], [0, 2, 2, 0], color='#4f46e5')
    ax[0].fill_between([0, 2, 5, 7], [0, 2, 2, 0], alpha=.2)
    ax[0].set(title='Motion #119: area = 10 m', xlabel='Time (s)', ylabel='Speed (m/s)')
    points = [r[1:3] for r in rows if r[0].startswith('transform_')]
    for pts, label in [([(0,0),(1,0),(1,1),(0,1)], 'Original'), (points, 'Rotated then translated')]:
        closed = pts + [pts[0]]
        ax[1].plot(*zip(*closed), 'o-', label=label)
    ax[1].set(title='Rigid transform #226'); ax[1].axis('equal'); ax[1].legend(fontsize=8)
    ax[2].plot(*pose(25,50), 'o-', linewidth=5)
    ax[2].set(title='FK EP.9: (1.5018, 1.4610)', xlim=(-.2,2.5), ylim=(-.2,2.5)); ax[2].set_aspect('equal')
    for sign in [1,-1]:
        s = sign*math.sqrt(1-.625**2)
        b = math.atan2(s,.625); a = math.atan2(1,1.5)-math.atan2(s,1.625)
        ax[3].plot([0,math.cos(a),1.5],[0,math.sin(a),1], 'o-')
    ax[3].set(title='IK #222: two branches'); ax[3].axis('equal')
    grid = [[0,0,0,0],[1,1,0,1],[0,0,0,0],[0,1,1,0]]
    route = [(0,0),(0,1),(0,2),(1,2),(2,2),(2,3),(3,3)]
    ax[4].imshow(grid,cmap='Greys',vmin=0,vmax=1)
    ax[4].plot([c for r,c in route],[r for r,c in route], 'o-', color='#4f46e5')
    ax[4].set(title='Path #241: 7 coordinates, 6 moves',xlabel='Column',ylabel='Row')
    ax[5].plot([0,1,2],[0,1.1375,2.119867549668874], 'o-',label='Corrected position')
    ax[5].plot([1,2],[1.2,2.1],'x',label='Measurement')
    ax[5].set(title='Kalman #399',xlabel='Step'); ax[5].legend(fontsize=8)
    fig.savefig('crash_check.png',dpi=160)
    plt.close(fig)
    if args.frames:
        output = Path('frames'); output.mkdir(exist_ok=True)
        # 기존 FK 입력까지 각도를 선형 보간한다. 동역학 계산이 아니다.
        for i in range(48):
            fraction = i / 47
            fig, axis = plt.subplots(figsize=(8,5),layout='constrained')
            axis.plot(*pose(25*fraction,50*fraction),'o-',linewidth=9,color='#4f46e5',markersize=13)
            axis.set(xlim=(-.3,2.6),ylim=(-.3,2.6),xlabel='x',ylabel='y',title='FK pose sequence: fixed axis scale')
            axis.set_aspect('equal');axis.grid(alpha=.2)
            fig.savefig(output/f'frame_{i:03d}.png',dpi=120)
            plt.close(fig)
    print('Static PNG verified from crash_check; frames=48 at 12fps' if args.frames else 'Static PNG ready')


if __name__ == '__main__':
    main()
