import argparse
import os
from dh_fk import sweep


def draw(rows, out, upto=None):
    import matplotlib.pyplot as plt
    shown = rows if upto is None else rows[:upto]
    x, y = zip(*[(row[1], row[2]) for row in shown])
    elbow = (2.0 * 0.8660254038, 2.0 * 0.5)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(x, y, "o-", color="#4f46e5", linewidth=2.5, label="trajectory")
    ax.add_patch(plt.Circle(elbow, 1.5, fill=False, color="#059669", linestyle="--"))
    ax.scatter(*elbow, color="#111827", zorder=3)
    ax.set_aspect("equal"); ax.grid(); ax.legend(); ax.set_xlabel("x"); ax.set_ylabel("y")
    fig.savefig(out, dpi=160, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", action="store_true")
    args = parser.parse_args()
    rows = sweep()
    draw(rows, "trajectory.png")
    if args.frames:
        os.makedirs("frames", exist_ok=True)
        for i in range(1, len(rows) + 1):
            draw(rows, f"frames/frame_{i - 1:02d}.png", i)
