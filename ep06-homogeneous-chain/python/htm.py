import csv
import math


def T4(R, t):
    return [R[i] + [t[i]] for i in range(3)] + [[0, 0, 0, 1]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def apply(T, p):
    q = [p[0], p[1], p[2], 1]
    return [round(sum(T[i][k] * q[k] for k in range(4)), 4) for i in range(3)]


def inv(T):
    Rt = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(Rt[i][k] * T[k][3] for k in range(3)) for i in range(3)]
    return T4(Rt, t)


def transform2d(x, y):
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    return round(c*x-s*y+1, 4), round(s*x+c*y+.5, 4)


if __name__ == "__main__":
    rz = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]; eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    ac = mul(T4(rz, [1, 0, 0]), T4(eye, [0, 2, 0])); p = apply(ac, [1, 0, 0])
    t = [round(ac[i][3], 4) for i in range(3)]; back = apply(inv(ac), p)
    print(t, p); print(back)
    assert t == [-1, 0, 0] and p == [-1, 1, 0] and back == [1, 0, 0]
    square = [transform2d(x, y) for x, y in [(0,0),(1,0),(1,1),(0,1)]]
    print(square)
    with open("chain.csv", "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows([["tx","ty","tz","px","py","pz"], t+p])
