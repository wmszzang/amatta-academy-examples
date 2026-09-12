"""지문 A~E + 경계 2문제(#512, #558)를 tf.py 로 풀어 제출 값을 소수 4자리로 출력한다.

python python/run_all.py > python-output.txt
(C·C++ 판과 바이트 단위로 같은 출력이어야 한다.)
"""
import math
import tf


def f(x):
    if abs(x) < 5e-5:
        x = 0.0  # -0.0000 방지
    return f"{x:.4f}"


def pt(p):
    return "(" + ",".join(f(v) for v in p) + ")"


# ---------- A. #226 팔레트 네 꼭짓점: 35도 회전 후 (0.6, -0.4) 이동 ----------
print("[A] #226 rigid2d rot=35deg t=(0.6,-0.4)")
pts = [(0.0, 0.0), (1.2, 0.0), (1.2, 0.8), (0.0, 0.8)]
ang, t = 35.0, (0.6, -0.4)
out = tf.rigid2d(pts, ang, t)
for i, (p, q) in enumerate(zip(pts, out)):
    print(f"P{i} {pt(p)} -> {pt(q)}")
R = tf.rot2(ang)
print(f"cos35={f(R[0][0])} sin35={f(R[1][0])} det={f(R[0][0] * R[1][1] - R[0][1] * R[1][0])} area=0.9600")
wrong = tf.rigid2d([(x + t[0], y + t[1]) for x, y in pts], ang, (0.0, 0.0))
print("translate-then-rotate (wrong): " + " ".join(pt(q) for q in wrong))
inv_ang, inv_t = tf.inv2d(ang, t)
print(f"inv2d: rot={f(inv_ang)}deg t'={pt(inv_t)}")
print("inv2d applied: " + " ".join(pt(q) for q in tf.rigid2d(out, inv_ang, inv_t)))
a1, a2 = tf.recover_angle(pts[1], out[1], t)
print(f"recover_angle from P1'={pt(out[1])}: atan2={f(a1)}deg acos={f(a2)}deg wrap(-325)={f(tf.wrap(-325.0))}deg")

# ---------- B. #258 동차변환 체인 T_AC = T_AB * T_BC ----------
print("[B] #258 chain T_AC = T_AB * T_BC")
T_AB = tf.homog(tf.rotz(60.0), (0.5, 0.2, 0.0))
T_BC = tf.homog(tf.rotx(90.0), (0.0, 0.4, 0.3))
p_C = (0.2, 0.1, 0.5)
T_AC = tf.mul(T_AB, T_BC)
print(f"t_AC = {pt((T_AC[0][3], T_AC[1][3], T_AC[2][3]))}")
print(f"naive sum (wrong) = {pt((0.5 + 0.0, 0.2 + 0.4, 0.0 + 0.3))}")
p_B = tf.apply(T_BC, p_C)
print(f"p_B = {pt(p_B)}")
print(f"p_A via T_AC = {pt(tf.apply(T_AC, p_C))}")
print(f"p_A via two steps = {pt(tf.apply(T_AB, p_B))}")
print(f"reversed order T_BC*T_AB (wrong) = {pt(tf.apply(tf.mul(T_BC, T_AB), p_C))}")
print(f"inv4(T_AC) p_A = {pt(tf.apply(tf.inv4(T_AC), tf.apply(T_AC, p_C)))}")

# ---------- C. #221 2링크 / #220 3링크 순기구학 ----------
print("[C] #221 fk links=(1.4,0.9) angles=(25,50)")
j = tf.fk((1.4, 0.9), (25.0, 50.0))
for i, p in enumerate(j):
    print(f"j{i} {pt(p)}")
print(f"cumulative = 25.0000/75.0000 reach = {f(math.hypot(*j[-1]))}")
print(f"absolute-angle mistake (wrong) = {pt((j[1][0] + 0.9 * math.cos(math.radians(50.0)), j[1][1] + 0.9 * math.sin(math.radians(50.0))))}")
print("[C] #220 fk links=(1.2,0.9,0.6) angles=(20,35,-25)")
j3 = tf.fk((1.2, 0.9, 0.6), (20.0, 35.0, -25.0))
for i, p in enumerate(j3):
    print(f"j{i} {pt(p)}")
print("cumulative = 20.0000/55.0000/30.0000")

# ---------- D. #237 DH 체인 ----------
print("[D] #237 dh chain a=(1.4,0.9) theta=(25,50)")
A1, A2 = tf.dh(1.4, 0.0, 0.0, 25.0), tf.dh(0.9, 0.0, 0.0, 50.0)
print(f"A1 R=[{f(A1[0][0])},{f(A1[0][1])};{f(A1[1][0])},{f(A1[1][1])}] t={pt((A1[0][3], A1[1][3]))}")
print(f"A2 R=[{f(A2[0][0])},{f(A2[0][1])};{f(A2[1][0])},{f(A2[1][1])}] t={pt((A2[0][3], A2[1][3]))}")
T = tf.dh_chain([(1.4, 0.0, 0.0, 25.0), (0.9, 0.0, 0.0, 50.0)])
print(f"T=A1*A2 -> {pt((T[0][3], T[1][3]))} yaw=atan2(T21,T11)={f(tf.yaw_of(T))}deg")
T237 = tf.mul(tf.dh(2.0, 0.0, 0.0, math.degrees(0.5236)), tf.dh(1.5, 0.0, 0.0, math.degrees(0.7854)))
print(f"original #237 a=(2.0,1.5) theta=(0.5236,0.7854)rad -> {pt((T237[0][3], T237[1][3]))}")

# ---------- E. #223 관절각 스윕 ----------
print("[E] #223 sweep theta1=25 theta2=0..90 step 15")
rows = tf.sweep((1.4, 0.9), 25.0, 0, 90, 15)
for a, p in rows:
    print(f"{a:02d} {pt(p)} r_from_j1={f(math.hypot(p[0] - j[1][0], p[1] - j[1][1]))} dist={f(math.hypot(*p))}")
print(f"points = {len(rows)} center j1 = {pt(j[1])} annulus inner=0.5000 outer=2.3000")

# ---------- 경계 #512 SAT ----------
print("[#512] SAT obb A=(0,0,0.6,0.4,0deg) B=(1.1,0.35,0.5,0.3,30deg)")
A = (0.0, 0.0, 0.6, 0.4, 0.0)
for cx in (1.1, 1.45):
    B = (cx, 0.35, 0.5, 0.3, 30.0)
    print(f"B cx={f(cx)} corners: " + " ".join(pt(c) for c in tf.obb_corners(B)))
    hit, rep = tf.sat_obb(A, B)
    for k, (i1, i2, ov) in enumerate(rep, 1):
        print(f"  axis{k} A=[{f(i1[0])},{f(i1[1])}] B=[{f(i2[0])},{f(i2[1])}] {'overlap' if ov else 'separated'}")
    print(f"  collide = {'True' if hit else 'False'}")

# ---------- 경계 #558 원형 평균 ----------
print("[#558] circular mean headings=(10,350,20)")
h = (10.0, 350.0, 20.0)
print(f"simple mean (wrong) = {f(sum(h) / len(h))}")
print(f"circular mean = {f(tf.circular_mean(h))}deg")
