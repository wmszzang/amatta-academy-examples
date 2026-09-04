"""cubic.csv · quintic.csv 를 그래프로 — 정적 이미지 + '그려지는' 애니메이션.

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3

먼저 계산 프로그램(어느 언어든)을 실행해 cubic.csv, quintic.csv 를 만든 뒤:
    python visualize.py                     → trajectory.png (각도·각속도·각가속도 3단, 3차 vs 5차)
    python visualize.py --animate           → trajectory.mp4 (ffmpeg 없으면 trajectory.gif)
    python visualize.py --frames out --fps 15 --seconds 12
                                            → out/ 폴더에 프레임 PNG 시퀀스(영상 편집용)
    python visualize.py --scurve            → scurve.png (scurve.csv: 속도·가속도·저크 3단)

영상(EP.3)에 삽입된 애니메이션은 정확히 이 스크립트의 --frames 모드 산출물입니다.
"""
import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt

INDIGO, EMERALD, ROSE = "#4F46E5", "#059669", "#E11D48"


def load(path, cols):
    out = {c: [] for c in cols}
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            for c in cols:
                out[c].append(float(row[c]))
    return out


def make_axes(labels, t_max):
    fig, axes = plt.subplots(3, 1, figsize=(12.8, 7.2), sharex=True)
    fig.patch.set_facecolor("#F7F8FC")
    for ax, lab in zip(axes, labels):
        ax.set_facecolor("#FFFFFF")
        ax.grid(True, color="#E2E8F0", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_ylabel(lab, color="#1E293B")
        ax.axhline(0, color="#94A3B8", linewidth=0.8)
    axes[-1].set_xlabel("time (s)", color="#1E293B")
    axes[0].set_xlim(0, t_max * 1.04)
    fig.tight_layout()
    return fig, axes


def set_limits(axes, c, q):
    for ax, key in zip(axes, ("q", "qd", "qdd")):
        lo = min(min(c[key]), min(q[key]))
        hi = max(max(c[key]), max(q[key]))
        pad = (hi - lo) * 0.12 or 1.0
        ax.set_ylim(lo - pad, hi + pad)


def draw_static(c, q, out="trajectory.png"):
    fig, axes = make_axes(["angle (deg)", "velocity (deg/s)", "accel (deg/s^2)"], max(c["t"]))
    set_limits(axes, c, q)
    for ax, key in zip(axes, ("q", "qd", "qdd")):
        ax.plot(c["t"], c[key], color=INDIGO, linewidth=3, label="cubic")
        ax.plot(q["t"], q[key], color=EMERALD, linewidth=3, label="quintic")
    axes[0].legend(loc="upper left")
    fig.savefig(out, dpi=100)
    print(f"저장: {out}")


def _progressive(c, q, total, on_frame):
    fig, axes = make_axes(["angle (deg)", "velocity (deg/s)", "accel (deg/s^2)"], max(c["t"]))
    set_limits(axes, c, q)
    lines, dots = [], []
    for ax in axes:
        lc, = ax.plot([], [], color=INDIGO, linewidth=3, label="cubic")
        lq, = ax.plot([], [], color=EMERALD, linewidth=3, label="quintic")
        dc, = ax.plot([], [], "o", color=INDIGO, markersize=7)
        dq, = ax.plot([], [], "o", color=EMERALD, markersize=7)
        lines.append((lc, lq))
        dots.append((dc, dq))
    axes[0].legend(loc="upper left")
    n = len(c["t"])
    for i in range(total):
        k = max(1, int(round((i + 1) / total * n)))
        for (lc, lq), (dc, dq), key in zip(lines, dots, ("q", "qd", "qdd")):
            lc.set_data(c["t"][:k], c[key][:k])
            lq.set_data(q["t"][:k], q[key][:k])
            dc.set_data([c["t"][k - 1]], [c[key][k - 1]])
            dq.set_data([q["t"][k - 1]], [q[key][k - 1]])
        on_frame(fig, i)
    return fig


def export_frames(c, q, out_dir, fps=15, seconds=12.0):
    os.makedirs(out_dir, exist_ok=True)
    total = int(fps * seconds)
    _progressive(c, q, total, lambda fig, i: fig.savefig(os.path.join(out_dir, f"frame_{i:04d}.png"), dpi=100))
    print(f"저장: {out_dir}/frame_0000.png … frame_{total - 1:04d}.png ({total}장, {fps}fps)")


def animate(c, q, out_mp4="trajectory.mp4", fps=25, seconds=8.0):
    from matplotlib.animation import FFMpegWriter, PillowWriter

    total = int(fps * seconds)
    frames = []

    def grab(fig, i):
        fig.canvas.draw()
        frames.append(fig.canvas.buffer_rgba().tobytes())

    fig = _progressive(c, q, total, grab)
    w, h = fig.canvas.get_width_height()
    try:
        writer = FFMpegWriter(fps=fps)
        out = out_mp4
    except (FileNotFoundError, RuntimeError):
        writer = PillowWriter(fps=fps)
        out = os.path.splitext(out_mp4)[0] + ".gif"
    import numpy as np
    fig2, ax2 = plt.subplots(figsize=(w / 100, h / 100))
    ax2.axis("off")
    im = ax2.imshow(np.frombuffer(frames[0], dtype=np.uint8).reshape(h, w, 4))
    fig2.subplots_adjust(0, 0, 1, 1)
    with writer.saving(fig2, out, dpi=100):
        for buf in frames:
            im.set_data(np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4))
            writer.grab_frame()
    print(f"저장: {out}")


def draw_scurve(path="scurve.csv", out="scurve.png"):
    d = load(path, ["t", "x", "v", "a", "j"])
    fig, axes = make_axes(["velocity (deg/s)", "accel (deg/s^2)", "jerk (deg/s^3)"], max(d["t"]))
    for ax, key, col in zip(axes, ("v", "a", "j"), (INDIGO, EMERALD, ROSE)):
        ax.plot(d["t"], d[key], color=col, linewidth=3)
        lo, hi = min(d[key]), max(d[key])
        pad = (hi - lo) * 0.12 or 1.0
        ax.set_ylim(lo - pad, hi + pad)
    fig.savefig(out, dpi=100)
    print(f"저장: {out}")


if __name__ == "__main__":
    matplotlib.use("Agg")
    p = argparse.ArgumentParser()
    p.add_argument("--cubic", default="cubic.csv")
    p.add_argument("--quintic", default="quintic.csv")
    p.add_argument("--animate", action="store_true")
    p.add_argument("--frames", default="")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--scurve", action="store_true")
    a = p.parse_args()
    if a.scurve:
        draw_scurve()
    else:
        c = load(a.cubic, ["t", "q", "qd", "qdd"])
        q = load(a.quintic, ["t", "q", "qd", "qdd"])
        if a.frames:
            export_frames(c, q, a.frames, fps=a.fps, seconds=a.seconds)
        elif a.animate:
            animate(c, q)
        else:
            draw_static(c, q)
