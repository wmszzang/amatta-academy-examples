"""profile.csv를 그래프로 — 정적 이미지 + '그려지는' 애니메이션.

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.2

먼저 계산 프로그램(어느 언어든)을 실행해 profile.csv를 만든 뒤:
    python visualize.py                     → trapezoid.png (정적 그래프)
    python visualize.py --animate           → trapezoid.mp4 (ffmpeg 없으면 trapezoid.gif)
    python visualize.py --frames out --fps 15 --seconds 12
                                            → out/ 폴더에 프레임 PNG 시퀀스(영상 편집용)

영상(EP.2)에 삽입된 애니메이션은 정확히 이 스크립트의 --frames 모드 산출물입니다.
"""
import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt


def load(path="profile.csv"):
    t, s, v = [], [], []
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            s.append(float(row["s"]))
            v.append(float(row["v"]))
    return t, s, v


def make_axes(t, s, v):
    fig, (ax_v, ax_s) = plt.subplots(2, 1, figsize=(12.8, 7.2), sharex=True)
    fig.patch.set_facecolor("#F7F8FC")
    for ax in (ax_v, ax_s):
        ax.set_facecolor("#FFFFFF")
        ax.grid(True, color="#E2E8F0", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
    ax_v.set_ylabel("velocity (deg/s)", color="#1E293B")
    ax_s.set_ylabel("distance (deg)", color="#1E293B")
    ax_s.set_xlabel("time (s)", color="#1E293B")
    ax_v.set_xlim(0, max(t) * 1.04)
    ax_v.set_ylim(0, max(v) * 1.15)
    ax_s.set_ylim(0, max(s) * 1.1)
    fig.tight_layout()
    return fig, ax_v, ax_s


def draw_static(t, s, v, out="trapezoid.png"):
    fig, ax_v, ax_s = make_axes(t, s, v)
    ax_v.plot(t, v, color="#4F46E5", linewidth=3)
    ax_v.fill_between(t, v, color="#4F46E5", alpha=0.15)
    ax_s.plot(t, s, color="#059669", linewidth=3)
    fig.savefig(out, dpi=100)
    print(f"저장: {out}")


def animate(t, s, v, out_mp4="trapezoid.mp4"):
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

    fig, ax_v, ax_s = make_axes(t, s, v)
    line_v, = ax_v.plot([], [], color="#4F46E5", linewidth=3)
    line_s, = ax_s.plot([], [], color="#059669", linewidth=3)
    dot_v, = ax_v.plot([], [], "o", color="#4F46E5", markersize=8)
    dot_s, = ax_s.plot([], [], "o", color="#059669", markersize=8)
    fill = [None]

    step = max(1, len(t) // 300)  # ~300프레임으로 압축

    def update(i):
        k = min(i * step + 1, len(t))
        line_v.set_data(t[:k], v[:k])
        line_s.set_data(t[:k], s[:k])
        dot_v.set_data([t[k - 1]], [v[k - 1]])
        dot_s.set_data([t[k - 1]], [s[k - 1]])
        if fill[0] is not None:
            fill[0].remove()
        fill[0] = ax_v.fill_between(t[:k], v[:k], color="#4F46E5", alpha=0.15)
        return line_v, line_s, dot_v, dot_s

    frames = (len(t) + step - 1) // step
    anim = FuncAnimation(fig, update, frames=frames, interval=40, blit=False)
    try:
        anim.save(out_mp4, writer=FFMpegWriter(fps=25))
        print(f"저장: {out_mp4}")
    except (FileNotFoundError, RuntimeError):
        gif = os.path.splitext(out_mp4)[0] + ".gif"
        anim.save(gif, writer=PillowWriter(fps=25))
        print(f"ffmpeg이 없어 GIF로 저장: {gif}")


def export_frames(t, s, v, out_dir, fps=15, seconds=12.0):
    os.makedirs(out_dir, exist_ok=True)
    total = int(fps * seconds)
    fig, ax_v, ax_s = make_axes(t, s, v)
    line_v, = ax_v.plot([], [], color="#4F46E5", linewidth=3)
    line_s, = ax_s.plot([], [], color="#059669", linewidth=3)
    dot_v, = ax_v.plot([], [], "o", color="#4F46E5", markersize=8)
    dot_s, = ax_s.plot([], [], "o", color="#059669", markersize=8)
    fill = [None]
    for i in range(total):
        k = max(1, int(round((i + 1) / total * len(t))))
        line_v.set_data(t[:k], v[:k])
        line_s.set_data(t[:k], s[:k])
        dot_v.set_data([t[k - 1]], [v[k - 1]])
        dot_s.set_data([t[k - 1]], [s[k - 1]])
        if fill[0] is not None:
            fill[0].remove()
        fill[0] = ax_v.fill_between(t[:k], v[:k], color="#4F46E5", alpha=0.15)
        fig.savefig(os.path.join(out_dir, f"frame_{i:04d}.png"), dpi=100)
    print(f"저장: {out_dir}/frame_0000.png … frame_{total - 1:04d}.png ({total}장, {fps}fps)")


if __name__ == "__main__":
    matplotlib.use("Agg")
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default="profile.csv")
    p.add_argument("--animate", action="store_true")
    p.add_argument("--frames", default="")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--seconds", type=float, default=12.0)
    a = p.parse_args()
    t, s, v = load(a.csv)
    if a.frames:
        export_frames(t, s, v, a.frames, fps=a.fps, seconds=a.seconds)
    elif a.animate:
        animate(t, s, v)
    else:
        draw_static(t, s, v)
