"""EP.13 실행 결과 시각화 — 정적 PNG 또는 12 fps 프레임 시퀀스.

matplotlib만 추가로 쓴다(영상 삽화 전용). 계산 자체는 표준 math 모듈이다.
"""
import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from numeric_ik import (fk2, fk3, jacobian3, jacobian, solve, pinv_vel,
                        null_dir, mul, norm)
from statics import gravity_torque

MODES = ('newton', 'lam', 'null', 'statics')
BG = '#F4F6FB'
BLUE = '#4F46E5'
GOLD = '#D97706'
GREEN = '#059669'
ROSE = '#E11D48'
FRAMES = 36


def _fig():
    fig = plt.figure(figsize=(12, 6.75), dpi=100, facecolor=BG)
    return fig


def _arm2(ax, th, l1=2.0, l2=1.5, color=BLUE, alpha=1.0, lw=4):
    el = (l1 * math.cos(th[0]), l1 * math.sin(th[0]))
    end = fk2(l1, l2, th[0], th[1])
    ax.plot([0, el[0], end[0]], [0, el[1], end[1]], 'o-', linewidth=lw,
            color=color, alpha=alpha, markersize=6)
    return end


def _arm3(ax, th, a=(2.0, 1.5, 1.0), color=BLUE, alpha=1.0, lw=4):
    cum = [th[0], th[0] + th[1], th[0] + th[1] + th[2]]
    xs, ys = [0.0], [0.0]
    for i in range(3):
        xs.append(xs[-1] + a[i] * math.cos(cum[i]))
        ys.append(ys[-1] + a[i] * math.sin(cum[i]))
    ax.plot(xs, ys, 'o-', linewidth=lw, color=color, alpha=alpha, markersize=6)
    return xs[-1], ys[-1]


def render(mode, out, frames=False):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    total = FRAMES if frames else 1

    if mode == 'newton':
        run = solve((2.5, 1.5), (0.5, 0.5))
        errs = run['errors']
        poses = [(0.5, 0.5)]
        th = (0.5, 0.5)
        from numeric_ik import newton_step
        for _ in range(run['updates']):
            th, _e = newton_step((2.5, 1.5), th, 2.0, 1.5, 1.0)
            poses.append(th)
    elif mode == 'null':
        a3, th3 = (2.0, 1.5, 1.0), (0.5, 0.5, 0.5)
        J3 = jacobian3(a3, th3)
        n = null_dir(J3)

    for frame in range(total):
        progress = (frame + 1) / total if frames else 1.0
        fig = _fig()

        if mode == 'newton':
            ax = fig.add_subplot(121)
            ax.set_facecolor(BG); ax.set_aspect('equal')
            ax.set_xlim(-1, 4); ax.set_ylim(-1, 4); ax.grid(alpha=.2)
            ax.set_title('Newton-Raphson iterations', fontsize=17)
            shown = max(1, int(round(len(poses) * progress)))
            for i, p in enumerate(poses[:shown]):
                last = (i == shown - 1)
                _arm2(ax, p, color=BLUE if last else GOLD,
                      alpha=1.0 if last else 0.22, lw=5 if last else 3)
            ax.plot([2.5], [1.5], '*', color=ROSE, markersize=18)
            ax2 = fig.add_subplot(122)
            ax2.set_facecolor(BG); ax2.grid(alpha=.2)
            ax2.set_yscale('log'); ax2.set_xlabel('update'); ax2.set_ylabel('|e| (log scale)')
            ax2.set_title('error on a log axis', fontsize=17)
            k = max(2, int(round(len(errs) * progress)))
            ax2.plot(range(k), errs[:k], 'o-', color=BLUE, linewidth=2)
            ax2.set_xlim(-0.3, len(errs) - 0.7); ax2.set_ylim(1e-13, 3)

        elif mode == 'lam':
            lams = (0.01, 0.05, 0.1, 0.3, 1.0)
            counts = [solve((3.4, 0.15), (0.0, 0.0), lam=l)['updates'] for l in lams]
            ax = fig.add_subplot(111)
            ax.set_facecolor(BG); ax.grid(alpha=.2, axis='y')
            ax.set_title('DLS lambda sweep — updates to converge', fontsize=17)
            ax.set_xlabel('lambda'); ax.set_ylabel('updates')
            heights = [c * progress for c in counts]
            bars = ax.bar([str(l) for l in lams], heights,
                          color=[GREEN if c <= 10 else GOLD for c in counts])
            for b, c in zip(bars, counts):
                if progress > 0.95:
                    ax.text(b.get_x() + b.get_width() / 2, c + 3, str(c),
                            ha='center', fontsize=14)
            ax.set_ylim(0, 155)

        elif mode == 'null':
            alpha = 1.6 * math.sin(2 * math.pi * progress)
            q = [pinv_vel(J3, (0.0, 1.0))[k] + alpha * n[k] for k in range(3)]
            th_now = tuple(th3[k] + 0.30 * q[k] for k in range(3))
            ax = fig.add_subplot(111)
            ax.set_facecolor(BG); ax.set_aspect('equal')
            ax.set_xlim(-1, 4.2); ax.set_ylim(-1.2, 4.2); ax.grid(alpha=.2)
            _arm3(ax, th3, color=GOLD, alpha=0.25, lw=3)
            end = _arm3(ax, th_now, color=BLUE, lw=5)
            base = fk3(a3, th3)
            ax.plot([base[0]], [base[1]], 'o', color=ROSE, markersize=13)
            xd = mul(J3, q)
            ax.set_title(f'null space motion — |qdot|={norm(q):.4f}, '
                         f'tip velocity=({xd[0]:.3f}, {xd[1]:.3f})', fontsize=16)

        elif mode == 'statics':
            ax = fig.add_subplot(111)
            ax.set_facecolor(BG); ax.grid(alpha=.2)
            ax.set_title('gravity compensation torque vs posture', fontsize=17)
            ax.set_xlabel('theta1 (deg), theta2 = 0.5 rad'); ax.set_ylabel('torque (N*m)')
            degs = [d for d in range(0, 91)]
            k = max(2, int(round(len(degs) * progress)))
            t1 = [gravity_torque(math.radians(d), 0.5)[0] for d in degs[:k]]
            t2 = [gravity_torque(math.radians(d), 0.5)[1] for d in degs[:k]]
            ax.plot(degs[:k], t1, color=BLUE, linewidth=3, label='joint 1')
            ax.plot(degs[:k], t2, color=GREEN, linewidth=3, label='joint 2')
            ax.set_xlim(0, 90); ax.set_ylim(-5, 90); ax.legend(fontsize=13)

        fig.tight_layout()
        name = f'{mode}-{frame:03d}.png' if frames else f'{mode}.png'
        fig.savefig(out / name, facecolor=BG)
        plt.close(fig)
    print(f'{mode}: {total}장 -> {out.resolve()}')


def main():
    ap = argparse.ArgumentParser()
    for m in MODES:
        ap.add_argument(f'--{m}', action='store_true')
    ap.add_argument('--frames', action='store_true')
    ap.add_argument('--out', default='output-python')
    args = ap.parse_args()
    chosen = [m for m in MODES if getattr(args, m)] or list(MODES)
    for m in chosen:
        render(m, Path(args.out) / (f'frames-{m}' if args.frames else '.'), args.frames)


if __name__ == '__main__':
    main()
