"""EP.14 실행 결과 시각화 — 정적 PNG 또는 12 fps 프레임 시퀀스.

matplotlib만 추가로 쓴다(영상 삽화 전용). 계산 자체는 표준 math 모듈이다.
축 라벨은 영문으로 둔다(폰트 의존을 없애려는 의도).

  python visualize.py                 # 여섯 장면 정적 PNG
  python visualize.py --frames        # 장면마다 frames-<mode>/ 프레임 시퀀스
  python visualize.py --case ik       # 한 장면만
"""
import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from kin14 import (det2, fk2, fk_n, hull, ik2, inv_jac_qdot, jac, jac_n,
                   jpinv_ik3, newton_dls, null_dir, point_in_polygon,
                   shoelace, sweep_points, torque, velocity)

MODES = ('ik', 'hull', 'vel', 'burst', 'null', 'panel')
BG = '#F4F6FB'
BLUE = '#4F46E5'
GOLD = '#D97706'
GREEN = '#059669'
ROSE = '#E11D48'
FRAMES = 36
FENCE = [(-1.8, -0.4), (3.8, -0.4), (3.8, 2.4), (1.0, 3.8), (-1.8, 2.4)]


def _arm2(ax, th, a1=2.0, a2=1.5, color=BLUE, alpha=1.0, lw=4):
    el = (a1 * math.cos(th[0]), a1 * math.sin(th[0]))
    end = fk2(a1, a2, th[0], th[1])
    ax.plot([0, el[0], end[0]], [0, el[1], end[1]], 'o-', linewidth=lw,
            color=color, alpha=alpha, markersize=6)
    return end


def _arm3(ax, th, lens=(2.0, 1.5, 1.0), color=BLUE, alpha=1.0, lw=4):
    xs, ys, cum, x, y = [0.0], [0.0], 0.0, 0.0, 0.0
    for l, t in zip(lens, th):
        cum += t
        x += l * math.cos(cum)
        y += l * math.sin(cum)
        xs.append(x)
        ys.append(y)
    ax.plot(xs, ys, 'o-', linewidth=lw, color=color, alpha=alpha, markersize=6)
    return (x, y)


def _frame(mode, ax_maker, progress):
    pass


def render(mode, out, frames=False):
    out.mkdir(parents=True, exist_ok=True)
    total = FRAMES if frames else 1
    for frame in range(total):
        p = (frame + 1) / total if frames else 1.0
        fig = plt.figure(figsize=(12, 6.75), dpi=100, facecolor=BG)

        if mode == 'ik':                       # 두 해 스틱 다이어그램
            ax = fig.add_subplot(111)
            down = ik2(2.0, 1.5, 2.5, 1.5, 'down')
            up = ik2(2.0, 1.5, 2.5, 1.5, 'up')
            k = 0.5 - 0.5 * math.cos(math.pi * p)          # 0 -> 1 부드럽게
            _arm2(ax, down, color=BLUE, alpha=0.35, lw=3)
            _arm2(ax, up, color=GREEN, alpha=0.35, lw=3)
            mid = (down[0] + (up[0] - down[0]) * k, down[1] + (up[1] - down[1]) * k)
            _arm2(ax, mid, color=GOLD, lw=5)
            ax.plot([2.5], [1.5], '*', markersize=22, color=ROSE)
            ax.set_title('two IK branches reach the same target (2.5, 1.5)', fontsize=17)
            ax.set_xlim(-0.6, 3.8); ax.set_ylim(-0.8, 3.2)
            ax.set_aspect('equal'); ax.grid(alpha=.3)
            ax.text(0.05, 0.94, 'elbow-down (0.0432, 1.1864) rad', transform=ax.transAxes,
                    color=BLUE, fontsize=14)
            ax.text(0.05, 0.88, 'elbow-up  (1.0376, -1.1864) rad', transform=ax.transAxes,
                    color=GREEN, fontsize=14)

        elif mode == 'hull':                   # 껍질이 닫히는 과정 + 펜스
            ax = fig.add_subplot(111)
            pts = sweep_points(2.0, 1.5, (0, 90), (0, 120), 25, 25)
            H = hull(pts)
            ax.plot([q[0] for q in pts], [q[1] for q in pts], '.', color=BLUE,
                    markersize=3, alpha=.5)
            n = max(2, int(len(H) * p))
            ring = H[:n] + ([H[0]] if n == len(H) else [])
            ax.plot([q[0] for q in ring], [q[1] for q in ring], '-o', color=GOLD,
                    linewidth=3, markersize=5)
            fx = [q[0] for q in FENCE] + [FENCE[0][0]]
            fy = [q[1] for q in FENCE] + [FENCE[0][1]]
            ax.plot(fx, fy, '--', color=ROSE, linewidth=2)
            out_pts = [q for q in pts if not point_in_polygon(q, FENCE)]
            ax.plot([q[0] for q in out_pts], [q[1] for q in out_pts], 'x',
                    color=ROSE, markersize=5)
            ax.set_title('monotone-chain hull %d/%d vertices | fence check'
                         % (n, len(H)), fontsize=17)
            ax.set_xlim(-2.2, 4.2); ax.set_ylim(-0.9, 4.1)
            ax.set_aspect('equal'); ax.grid(alpha=.3)
            ax.text(0.02, 0.05, 'hull area 11.0790 vs occupancy 7.1523 m2 | outside 56/625',
                    transform=ax.transAxes, fontsize=13, color=ROSE)

        elif mode == 'vel':                    # 속도 벡터 + 중앙차분
            ax = fig.add_subplot(111)
            q = (0.5, 0.5)
            J = jac(2.0, 1.5, *q)
            v = velocity(J, (0.4, -0.3))
            tip = _arm2(ax, q, color=BLUE, lw=5)
            ax.arrow(tip[0], tip[1], v[0] * p, v[1] * p, head_width=.09,
                     color=GOLD, length_includes_head=True, linewidth=2)
            F = (3.0, -1.0)
            ax.arrow(tip[0], tip[1], F[0] * .18 * p, F[1] * .18 * p, head_width=.09,
                     color=ROSE, length_includes_head=True, linewidth=2)
            ax.set_title('v = J qdot (gold, m/s) and applied force F (rose, N)', fontsize=17)
            ax.set_xlim(-0.4, 3.4); ax.set_ylim(-0.8, 3.0)
            ax.set_aspect('equal'); ax.grid(alpha=.3)
            tau = torque(J, F)
            ax.text(0.02, 0.94, 'v = (-0.5098, 0.7831) m/s   |v| = 0.9344',
                    transform=ax.transAxes, fontsize=14, color=GOLD)
            ax.text(0.02, 0.88, 'tau = J^T F = (%.4f, %.4f) N.m' % tau,
                    transform=ax.transAxes, fontsize=14, color=ROSE)

        elif mode == 'burst':                  # |qdot| 발산 로그 그래프
            ax = fig.add_subplot(111)
            xs = [0.5, 0.2, 0.05, 0.01, 0.001]
            ys = []
            for t2 in xs:
                Ji = jac(2.0, 1.5, 0.5, t2)
                dq = inv_jac_qdot(Ji, (0.0, 0.2))
                ys.append(math.hypot(*dq))
            n = max(2, int(len(xs) * p))
            ax.plot(xs[:n], ys[:n], '-o', color=ROSE, linewidth=3, markersize=9)
            ax.set_xscale('log'); ax.set_yscale('log')
            ax.invert_xaxis()
            ax.set_xlabel('theta2 (rad)', fontsize=14)
            ax.set_ylabel('required |qdot| (rad/s)', fontsize=14)
            ax.set_title('same 0.2 m/s target, joint speed blows up as det J -> 0',
                         fontsize=17)
            ax.grid(alpha=.3, which='both')
            for i in range(n):
                ax.annotate('%.4f' % ys[i], (xs[i], ys[i]), fontsize=12,
                            textcoords='offset points', xytext=(6, 8))

        elif mode == 'null':                   # 널공간 이동 시 손끝 이탈
            ax = fig.add_subplot(111)
            lens = (2.0, 1.5, 1.0)
            r3 = jpinv_ik3((2.6, 1.6), (0.3, 0.4, 0.2), lens)
            n = null_dir(jac_n(lens, r3['th']))
            step = 0.15 * p
            moved = tuple(r3['th'][i] + step * n[i] for i in range(3))
            _arm3(ax, r3['th'], lens, color=BLUE, alpha=.35, lw=3)
            tip = _arm3(ax, moved, lens, color=GOLD, lw=5)
            ax.plot([2.6], [1.6], '*', markersize=22, color=ROSE)
            ax.set_title('null-space motion: elbow moves, tip drifts %.6f m'
                         % math.hypot(tip[0] - 2.6, tip[1] - 1.6), fontsize=17)
            ax.set_xlim(-0.8, 3.6); ax.set_ylim(-0.6, 3.4)
            ax.set_aspect('equal'); ax.grid(alpha=.3)
            ax.text(0.02, 0.94, 'n = (0.1688, -0.4955, 0.8520)', transform=ax.transAxes,
                    fontsize=14, color=GOLD)

        elif mode == 'panel':                  # 4패널 종합
            k = max(1, int(4 * p + 0.999))
            titles = ['(1) two IK branches', '(2) hull and fence',
                      '(3) velocity and force', '(4) convergence log']
            for idx in range(4):
                ax = fig.add_subplot(2, 2, idx + 1)
                ax.set_title(titles[idx], fontsize=13)
                ax.grid(alpha=.3)
                if idx >= k:
                    ax.set_xticks([]); ax.set_yticks([])
                    continue
                if idx == 0:
                    _arm2(ax, ik2(2.0, 1.5, 2.5, 1.5, 'down'), color=BLUE, lw=3)
                    _arm2(ax, ik2(2.0, 1.5, 2.5, 1.5, 'up'), color=GREEN, lw=3)
                    ax.plot([2.5], [1.5], '*', markersize=14, color=ROSE)
                    ax.set_aspect('equal')
                elif idx == 1:
                    pts = sweep_points(2.0, 1.5, (0, 90), (0, 120), 25, 25)
                    H = hull(pts) + [hull(pts)[0]]
                    ax.plot([q[0] for q in pts], [q[1] for q in pts], '.',
                            color=BLUE, markersize=2, alpha=.5)
                    ax.plot([q[0] for q in H], [q[1] for q in H], '-', color=GOLD, linewidth=2)
                    ax.plot([q[0] for q in FENCE] + [FENCE[0][0]],
                            [q[1] for q in FENCE] + [FENCE[0][1]], '--', color=ROSE)
                    ax.set_aspect('equal')
                elif idx == 2:
                    q = (0.5, 0.5)
                    J = jac(2.0, 1.5, *q)
                    v = velocity(J, (0.4, -0.3))
                    tip = _arm2(ax, q, color=BLUE, lw=3)
                    ax.arrow(tip[0], tip[1], v[0], v[1], head_width=.08, color=GOLD,
                             length_includes_head=True)
                    ax.arrow(tip[0], tip[1], .54, -.18, head_width=.08, color=ROSE,
                             length_includes_head=True)
                    ax.set_aspect('equal')
                else:
                    errs = newton_dls((2.5, 1.5), (0.5, 0.5), 2.0, 1.5)['errs']
                    dls = newton_dls((2.5, 1.5), (0.0, 0.0), 2.0, 1.5, lam=0.05)['errs']
                    ax.semilogy(range(1, len(errs) + 1), errs, '-o', color=BLUE,
                                label='Newton (6 checks)')
                    ax.semilogy(range(1, len(dls) + 1), dls, '-s', color=GOLD,
                                label='DLS lam=0.05 (8 checks)')
                    ax.set_xlabel('error check number', fontsize=11)
                    ax.legend(fontsize=10)
            fig.suptitle('EP.14 submission set: numbers plus the requested plots',
                         fontsize=16)

        fig.tight_layout()
        name = '%s-%03d.png' % (mode, frame) if frames else '%s.png' % mode
        fig.savefig(out / name, facecolor=BG)
        plt.close(fig)
    print('%s -> %s (%d장)' % (mode, out, total))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='output-python')
    ap.add_argument('--frames', action='store_true')
    ap.add_argument('--case', default=None, choices=MODES)
    args = ap.parse_args()
    for m in ([args.case] if args.case else MODES):
        base = Path(args.out) / ('frames-%s' % m if args.frames else '.')
        render(m, base, args.frames)
