"""motion_wrapup.py 가 만든 CSV 4개를 그래프로 그린다 (matplotlib).

Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.4

사용:
    python visualize.py                       정적 그래프 → motion_wrapup.png (4행: 지문 ①~④ × 3열: 위치·속도·가속도)
    python visualize.py --profiles            속도 프로파일 4장 비교 → profiles.png (사다리꼴 · 종 · 종(5차) · 둥근 사다리꼴)
    python visualize.py --animate             속도 프로파일이 그려지는 애니메이션 → profiles.mp4 (ffmpeg 없으면 .gif)
    python visualize.py --frames out --fps 15 --seconds 12
                                              같은 애니메이션을 PNG 프레임으로 (영상 삽화용)

먼저 motion_wrapup.py 를 실행해 p1_trapezoid.csv · p2_cubic.csv · p3_quintic.csv · p4_scurve.csv 를 만들어 두세요.
영상(EP.4)에 삽입된 애니메이션은 정확히 이 스크립트의 --frames 모드 산출물입니다.
"""
import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

PROMPTS = [  # (파일, 제목, 열 이름 3개)
    ("p1_trapezoid.csv", "Prompt 1  trapezoid  (D=150, V=60, A=120)", ("x", "v", "a")),
    ("p2_cubic.csv", "Prompt 2  cubic  (30 -> 120 deg, T=3 s)", ("q", "qd", "qdd")),
    ("p3_quintic.csv", "Prompt 3  quintic  (-45 -> 45 deg, T=4 s)", ("q", "qd", "qdd")),
    ("p4_scurve.csv", "Prompt 4  S-curve  (D=180, V=90, A=180, J=720)", ("x", "v", "a")),
]
COLORS = ["#0EA5E9", "#4F46E5", "#10B981", "#F59E0B"]


def load(name):
    with open(name, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        rows = list(r)
    return {k: np.array([float(row[k]) for row in rows]) for k in rows[0].keys()}


def static(out="motion_wrapup.png"):
    fig, axes = plt.subplots(4, 3, figsize=(12.8, 9.6))
    for i, (fn, title, cols) in enumerate(PROMPTS):
        d = load(fn)
        for j, (col, lab) in enumerate(zip(cols, ("position [deg]", "velocity [deg/s]", "acceleration [deg/s^2]"))):
            ax = axes[i][j]
            ax.plot(d["t"], d[col], color=COLORS[i], lw=2)
            ax.grid(alpha=0.3)
            if j == 0:
                ax.set_ylabel(f"P{i + 1}")
            if i == 0:
                ax.set_title(lab, fontsize=10, pad=18 if j == 1 else 6)
            if i == 3:
                ax.set_xlabel("time [s]")
        axes[i][1].text(0.5, 1.02, title, transform=axes[i][1].transAxes, ha="center", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    print(f"저장: {out}")


def _profile_axes():
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 7.2))
    axes = axes.ravel()
    data = []
    for i, (fn, title, cols) in enumerate(PROMPTS):
        d = load(fn)
        ax = axes[i]
        ax.set_title(title, fontsize=10)
        ax.set_xlim(0, d["t"][-1] * 1.02)
        ax.set_ylim(0, max(d[cols[1]]) * 1.15)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("velocity [deg/s]")
        ax.grid(alpha=0.3)
        data.append((d["t"], d[cols[1]]))
    fig.tight_layout()
    return fig, axes, data


def profiles(out="profiles.png"):
    fig, axes, data = _profile_axes()
    for i, (t, v) in enumerate(data):
        axes[i].plot(t, v, color=COLORS[i], lw=2.5)
    fig.savefig(out, dpi=100)
    print(f"저장: {out}")


def _progressive(total, draw):
    """total 프레임 동안 네 프로파일이 왼쪽에서 오른쪽으로 동시에 그려진다."""
    fig, axes, data = _profile_axes()
    lines = [axes[i].plot([], [], color=COLORS[i], lw=2.5)[0] for i in range(4)]
    for k in range(total):
        frac = min(1.0, (k + 1) / (total * 0.85))       # 85% 지점에서 완성, 나머지는 정지
        for i, (t, v) in enumerate(data):
            n = max(2, int(len(t) * frac))
            lines[i].set_data(t[:n], v[:n])
        draw(fig, k)
    plt.close(fig)


def export_frames(out_dir, fps=15, seconds=12.0):
    os.makedirs(out_dir, exist_ok=True)
    total = int(fps * seconds)
    _progressive(total, lambda fig, k: fig.savefig(os.path.join(out_dir, f"frame_{k:04d}.png"), dpi=100))
    print(f"저장: {out_dir}/frame_0000.png … ({total}장)")


def animate(out="profiles.mp4", fps=15, seconds=12.0):
    total = int(fps * seconds)
    frames = []

    def grab(fig, k):
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba()).copy())

    _progressive(total, grab)
    from matplotlib import animation
    h, w = frames[0].shape[:2]
    fig2, ax2 = plt.subplots(figsize=(w / 100, h / 100))
    ax2.axis("off")
    fig2.subplots_adjust(0, 0, 1, 1)
    im = ax2.imshow(frames[0])
    try:
        writer = animation.FFMpegWriter(fps=fps)
    except Exception:
        out = os.path.splitext(out)[0] + ".gif"
        writer = animation.PillowWriter(fps=fps)
    with writer.saving(fig2, out, dpi=100):
        for buf in frames:
            im.set_data(buf)
            writer.grab_frame()
    print(f"저장: {out}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--profiles", action="store_true")
    p.add_argument("--animate", action="store_true")
    p.add_argument("--frames", default="")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--seconds", type=float, default=12.0)
    a = p.parse_args()
    if a.frames:
        matplotlib.use("Agg")
        export_frames(a.frames, fps=a.fps, seconds=a.seconds)
    elif a.animate:
        matplotlib.use("Agg")
        animate(fps=a.fps, seconds=a.seconds)
    elif a.profiles:
        profiles()
    else:
        static()
