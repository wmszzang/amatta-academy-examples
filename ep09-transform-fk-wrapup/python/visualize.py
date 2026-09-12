"""제출용 2행 3열 그림: 점 변환 · 프레임 체인 · 2/3링크 스틱 · 직접식 vs DH · 스윕 궤적 · 상자 정사영.

python python/visualize.py  → wrapup.png (모든 축 equal aspect)
"""
import math
import matplotlib.pyplot as plt
import tf


def closed(pts):
    return [p[0] for p in pts] + [pts[0][0]], [p[1] for p in pts] + [pts[0][1]]


fig, axes = plt.subplots(2, 3, figsize=(15, 9), constrained_layout=True)

# A. 점 변환
ax = axes[0][0]
pts = [(0.0, 0.0), (1.2, 0.0), (1.2, 0.8), (0.0, 0.8)]
out = tf.rigid2d(pts, 35.0, (0.6, -0.4))
ax.plot(*closed(pts), "o--", color="#9ca3af", label="original")
ax.plot(*closed(out), "o-", color="#4f46e5", label="rot 35 + t")
ax.set_title("A. rigid2d")

# B. 프레임 체인 (xy 평면 투영)
ax = axes[0][1]
T_AB = tf.homog(tf.rotz(60.0), (0.5, 0.2, 0.0))
T_AC = tf.mul(T_AB, tf.homog(tf.rotx(90.0), (0.0, 0.4, 0.3)))
for name, T, color in (("A", None, "#111827"), ("B", T_AB, "#059669"), ("C", T_AC, "#4f46e5")):
    o = (0.0, 0.0) if T is None else (T[0][3], T[1][3])
    xdir = (1.0, 0.0) if T is None else (T[0][0], T[1][0])
    ax.plot([o[0], o[0] + 0.3 * xdir[0]], [o[1], o[1] + 0.3 * xdir[1]], "-", color=color, linewidth=2)
    ax.plot(o[0], o[1], "o", color=color, label=f"frame {name}")
p_A = tf.apply(T_AC, (0.2, 0.1, 0.5))
ax.plot(p_A[0], p_A[1], "o", color="#dc2626", label="p_A")
ax.set_title("B. chain (xy)")

# C. 2링크 · 3링크 스틱
ax = axes[0][2]
j2 = tf.fk((1.4, 0.9), (25.0, 50.0))
j3 = tf.fk((1.2, 0.9, 0.6), (20.0, 35.0, -25.0))
ax.plot([p[0] for p in j2], [p[1] for p in j2], "o-", color="#4f46e5", linewidth=3, label="2-link")
ax.plot([p[0] for p in j3], [p[1] for p in j3], "s--", color="#059669", linewidth=2, label="3-link")
ax.set_title("C. fk stick")

# D. 직접식 vs DH
ax = axes[1][0]
T = tf.dh_chain([(1.4, 0.0, 0.0, 25.0), (0.9, 0.0, 0.0, 50.0)])
ax.plot([p[0] for p in j2], [p[1] for p in j2], "o-", color="#9ca3af", label="direct")
ax.plot(T[0][3], T[1][3], "x", color="#dc2626", markersize=14, label="DH A1*A2")
ax.set_title("D. direct vs DH")

# E. 스윕 궤적
ax = axes[1][1]
rows = tf.sweep((1.4, 0.9), 25.0, 0, 90, 15)
ax.plot([p[0] for _, p in rows], [p[1] for _, p in rows], "o-", color="#4f46e5", label="sweep")
ax.add_patch(plt.Circle(j2[1], 0.9, fill=False, color="#059669", linestyle="--"))
ax.plot(*j2[1], "o", color="#111827")
ax.set_title("E. sweep")

# #512 상자 정사영
ax = axes[1][2]
A = (0.0, 0.0, 0.6, 0.4, 0.0)
B = (1.1, 0.35, 0.5, 0.3, 30.0)
ax.plot(*closed(tf.obb_corners(A)), "-", color="#4f46e5", label="A")
ax.plot(*closed(tf.obb_corners(B)), "-", color="#059669", label="B")
hit, _ = tf.sat_obb(A, B)
ax.set_title(f"#512 SAT collide={hit}")

for ax in axes.flat:
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend(fontsize=8)
fig.savefig("wrapup.png", dpi=140)
print("saved wrapup.png")
