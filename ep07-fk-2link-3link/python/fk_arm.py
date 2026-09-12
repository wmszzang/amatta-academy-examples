import csv
import math


def fk(thetas_deg, lengths):
    points, x, y, acc = [(0.0, 0.0)], 0.0, 0.0, 0.0
    for theta, length in zip(thetas_deg, lengths):
        acc += math.radians(theta)
        x += length * math.cos(acc)
        y += length * math.sin(acc)
        points.append((x, y))
    return points


def write_csv(points, path="arm.csv"):
    with open(path, "w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
        writer.writerow(["x", "y"])
        writer.writerows((f"{x:.4f}", f"{y:.4f}") for x, y in points)


if __name__ == "__main__":
    for label, angles, lengths in (("2-link", [30, 45], [1.5, 1.0]), ("3-link", [15, 20, 30], [1.5, 1.2, 0.8])):
        points = fk(angles, lengths)
        print(label, " ".join(f"({x:.4f},{y:.4f})" for x, y in points))
    write_csv(fk([30, 45], [1.5, 1.0]))
