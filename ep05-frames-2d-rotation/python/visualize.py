"""frames2d.py 가 만든 points.csv 를 그래프로 그린다 (matplotlib).

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.5

사용:
    python visualize.py                 정적 그래프 → frames2d.png (원본 정사각형 vs 변환 후, equal aspect)
    python visualize.py --frames        회전 0→30° (6장) + 이동 0→(1, 0.5) (6장) = 12장 → frames/frame_00.png … frame_11.png
                                        + 각도 래핑 다이얼 → wrap_dial.png  (영상 삽화용, 1920×1080)
    python visualize.py --frames --out frames --deg 30 --tx 1 --ty 0.5

먼저 frames2d.py 를 실행해 points.csv 를 만들어 두세요. 영상(EP.5)에 삽입된 애니메이션은 정확히 --frames 모드 산출물입니다.
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ORIG = "#9CA3AF"     # 원본: 회색 점선
MOVED = "#4F46E5"    # 변환 후: 인디고
AXIS = (-0.3, 2.2)


def load(name="points.csv"):
    with open(name, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    orig = [(float(r["x"]), float(r["y"])) for r in rows]
    moved = [(float(r["xr"]), float(r["yr"])) for r in rows]
    return orig, moved


def closed(pts):
    xs = [p[0] for p in pts] + [pts[0][0]]
    ys = [p[1] for p in pts] + [pts[0][1]]
    return xs, ys


def rigid(points, deg, tx, ty):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return [(c * x - s * y + tx, s * x + c * y + ty) for x, y in points]


def setup_axes(ax):
    ax.set_xlim(*AXIS)
    ax.set_ylim(*AXIS)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.axhline(0, color="#1E293B", linewidth=1)
    ax.axvline(0, color="#1E293B", linewidth=1)
    ax.set_xlabel("x")
    ax.set_ylabel("y")


def static(out="frames2d.png"):
    orig, moved = load()
    fig, ax = plt.subplots(figsize=(8, 8))
    setup_axes(ax)
    ax.plot(*closed(orig), linestyle="--", color=ORIG, marker="o", label="original")
    ax.plot(*closed(moved), color=MOVED, marker="o", linewidth=2.5, label="transformed (R 30deg, t=(1, 0.5))")
    for x, y in moved:
        ax.annotate(f"({x:.3f}, {y:.3f})", (x, y), textcoords="offset points", xytext=(6, 6), fontsize=9, color=MOVED)
    ax.legend(loc="upper left")
    ax.set_title("2D rigid transform: rotate 30 deg, then move (1, 0.5)")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print("saved:", out)


def frames(out_dir="frames", deg=30.0, tx=1.0, ty=0.5, n_rot=6, n_move=6):
    """회전이 먼저 0→deg 로 진행되고(n_rot 장), 그다음 이동이 0→t 로 진행된다(n_move 장)."""
    orig, _ = load()
    os.makedirs(out_dir, exist_ok=True)
    steps = []
    for i in range(n_rot):
        steps.append((deg * i / (n_rot - 1), 0.0, 0.0, f"rotate {deg * i / (n_rot - 1):.0f} deg"))
    for i in range(1, n_move + 1):
        k = i / n_move
        steps.append((deg, tx * k, ty * k, f"move ({tx * k:.2f}, {ty * k:.2f})"))
    for idx, (d, x, y, label) in enumerate(steps):
        fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)
        setup_axes(ax)
        ax.plot(*closed(orig), linestyle="--", color=ORIG, marker="o", label="original")
        cur = rigid(orig, d, x, y)
        ax.plot(*closed(cur), color=MOVED, marker="o", linewidth=3, label=label)
        ax.legend(loc="upper left", fontsize=14)
        ax.tick_params(labelsize=12)
        fig.tight_layout()
        path = os.path.join(out_dir, f"frame_{idx:02d}.png")
        fig.savefig(path)
        plt.close(fig)
    print(f"saved: {out_dir}/frame_00.png .. frame_{len(steps) - 1:02d}.png ({len(steps)} frames)")


def wrap_dial(out="wrap_dial.png"):
    """359° 와 −1° 가 같은 방향임을 보여 주는 원형 다이얼."""
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)
    ax.set_aspect("equal")
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.3, 1.3)
    ax.axis("off")
    circle = plt.Circle((0, 0), 1.0, fill=False, linewidth=3, color="#1E293B")
    ax.add_patch(circle)
    for ang, txt in ((0, "0"), (90, "90"), (180, "180 / -180"), (270, "-90")):
        r = math.radians(ang)
        ax.plot([0.92 * math.cos(r), 1.0 * math.cos(r)], [0.92 * math.sin(r), 1.0 * math.sin(r)], color="#1E293B", linewidth=3)
        ax.text(1.14 * math.cos(r), 1.14 * math.sin(r), txt, ha="center", va="center", fontsize=18)
    for ang, col, txt in ((359, "#E11D48", "359 deg"), (-1, "#0284C7", "-1 deg")):
        r = math.radians(ang)
        ax.annotate("", xy=(0.88 * math.cos(r), 0.88 * math.sin(r)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color=col, lw=4))
        ax.text(1.3, 0.12 if ang > 0 else -0.12, txt, color=col, fontsize=22, va="center")
    ax.text(0, -1.22, "359 deg == -1 deg  (wrap to [-180, 180])", ha="center", fontsize=22)
    fig.savefig(out)
    plt.close(fig)
    print("saved:", out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", action="store_true")
    ap.add_argument("--out", default="frames")
    ap.add_argument("--deg", type=float, default=30.0)
    ap.add_argument("--tx", type=float, default=1.0)
    ap.add_argument("--ty", type=float, default=0.5)
    a = ap.parse_args()
    if a.frames:
        frames(a.out, a.deg, a.tx, a.ty)
        wrap_dial()
    else:
        static()
