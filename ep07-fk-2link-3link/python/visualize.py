import argparse
import csv
import os
import matplotlib.pyplot as plt


def load(path):
    with open(path, encoding="utf-8") as source:
        return [(float(row["x"]), float(row["y"])) for row in csv.DictReader(source)]


def draw(points, out):
    x, y = zip(*points)
    plt.figure(figsize=(7, 5)); plt.plot(x, y, "o-", linewidth=4)
    for i, point in enumerate(points): plt.annotate(f"P{i} ({point[0]:.2f}, {point[1]:.2f})", point)
    plt.grid(); plt.axis("equal"); plt.savefig(out, dpi=160, bbox_inches="tight")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(); parser.add_argument("--frames", action="store_true"); args = parser.parse_args()
    draw(load("arm.csv"), "arm.png")
    if args.frames:
        os.makedirs("frames", exist_ok=True)
        for i in range(12): draw(load("arm.csv"), f"frames/frame_{i:02d}.png")
