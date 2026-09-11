import csv
import math


def dh(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def mul(left, right):
    return [[sum(left[i][k] * right[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def fk_planar(theta2_deg):
    t1, t2 = math.radians(30.0), math.radians(theta2_deg)
    return 2.0 * math.cos(t1) + 1.5 * math.cos(t1 + t2), 2.0 * math.sin(t1) + 1.5 * math.sin(t1 + t2)


def sweep():
    return [(angle, *fk_planar(angle)) for angle in range(0, 91, 15)]


def write_csv(rows, path="traj.csv"):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["theta2_deg", "x", "y"])
        writer.writerows((angle, f"{x:.4f}", f"{y:.4f}") for angle, x, y in rows)


if __name__ == "__main__":
    matrix = mul(dh(2.0, 0.0, 0.0, 0.5236), dh(1.5, 0.0, 0.0, 0.7854))
    print(f"POSE {matrix[0][3]:.4f},{matrix[1][3]:.4f},{matrix[2][3]:.4f}")
    rows = sweep()
    for angle, x, y in rows:
        print(f"{angle:02d} {x:.4f},{y:.4f}")
    write_csv(rows)
