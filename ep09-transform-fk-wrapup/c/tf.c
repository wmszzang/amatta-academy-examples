/* EP.9 지문 A~E + 경계 2문제 — C 최소 구현 (외부 라이브러리 없음).
 * gcc c/tf.c -o tf-c.exe -lm; .\tf-c.exe   → python/run_all.py 와 같은 출력 */
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
static double rad(double deg) { return deg * PI / 180.0; }
static double deg(double r) { return r * 180.0 / PI; }

/* -0.0000 방지용 출력 도우미 */
static void pf(double x) { if (fabs(x) < 5e-5) x = 0.0; printf("%.4f", x); }
static void p2(double x, double y) { printf("("); pf(x); printf(","); pf(y); printf(")"); }
static void p3(double x, double y, double z) { printf("("); pf(x); printf(","); pf(y); printf(","); pf(z); printf(")"); }

/* ---------- 각도 도구 ---------- */
static double wrap(double d) { double r = fmod(d + 180.0, 360.0); if (r < 0) r += 360.0; return r - 180.0; }
static double circular_mean(const double *h, int n) {
    double s = 0, c = 0;
    for (int i = 0; i < n; ++i) { s += sin(rad(h[i])); c += cos(rad(h[i])); }
    return deg(atan2(s, c));
}

/* ---------- 2D 강체변환 ---------- */
static void rigid2d(double x, double y, double a, double tx, double ty, double *ox, double *oy) {
    double c = cos(rad(a)), s = sin(rad(a));
    *ox = c * x - s * y + tx; *oy = s * x + c * y + ty;
}
static void inv2d(double a, double tx, double ty, double *ia, double *itx, double *ity) {
    double c = cos(rad(a)), s = sin(rad(a));
    *ia = -a; *itx = -(c * tx + s * ty); *ity = -(-s * tx + c * ty);
}

/* ---------- 4x4 동차변환 ---------- */
typedef struct { double m[4][4]; } T4;
static T4 homog_z(double a, double tx, double ty, double tz) {
    double c = cos(rad(a)), s = sin(rad(a));
    T4 t = {{{c, -s, 0, tx}, {s, c, 0, ty}, {0, 0, 1, tz}, {0, 0, 0, 1}}}; return t;
}
static T4 homog_x(double a, double tx, double ty, double tz) {
    double c = cos(rad(a)), s = sin(rad(a));
    T4 t = {{{1, 0, 0, tx}, {0, c, -s, ty}, {0, s, c, tz}, {0, 0, 0, 1}}}; return t;
}
static T4 mul(T4 a, T4 b) {
    T4 o;
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
        o.m[i][j] = 0.0;
        for (int k = 0; k < 4; ++k) o.m[i][j] += a.m[i][k] * b.m[k][j];
    }
    return o;
}
static void apply(T4 t, const double p[3], double o[3]) {
    for (int i = 0; i < 3; ++i) o[i] = t.m[i][0] * p[0] + t.m[i][1] * p[1] + t.m[i][2] * p[2] + t.m[i][3];
}
static T4 inv4(T4 t) {
    T4 o;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) o.m[i][j] = t.m[j][i];
    for (int i = 0; i < 3; ++i) o.m[i][3] = -(o.m[i][0] * t.m[0][3] + o.m[i][1] * t.m[1][3] + o.m[i][2] * t.m[2][3]);
    o.m[3][0] = o.m[3][1] = o.m[3][2] = 0.0; o.m[3][3] = 1.0;
    return o;
}

/* ---------- 다링크 순기구학 ---------- */
static void fk(const double *links, const double *angles, int n, double xs[], double ys[]) {
    double x = 0, y = 0, cum = 0;
    xs[0] = 0.0; ys[0] = 0.0;
    for (int i = 0; i < n; ++i) {
        cum += angles[i];
        x += links[i] * cos(rad(cum)); y += links[i] * sin(rad(cum));
        xs[i + 1] = x; ys[i + 1] = y;
    }
}

/* ---------- DH ---------- */
static T4 dh(double a, double alpha, double d, double theta) {
    double ca = cos(rad(alpha)), sa = sin(rad(alpha)), ct = cos(rad(theta)), st = sin(rad(theta));
    T4 t = {{{ct, -st * ca, st * sa, a * ct}, {st, ct * ca, -ct * sa, a * st}, {0, sa, ca, d}, {0, 0, 0, 1}}};
    return t;
}

/* ---------- SAT ---------- */
typedef struct { double cx, cy, hw, hh, deg; } OBB;
static void obb_corners(OBB b, double xs[4], double ys[4]) {
    static const double sx[4] = {-1, 1, 1, -1}, sy[4] = {-1, -1, 1, 1};
    double c = cos(rad(b.deg)), s = sin(rad(b.deg));
    for (int i = 0; i < 4; ++i) {
        double lx = sx[i] * b.hw, ly = sy[i] * b.hh;
        xs[i] = b.cx + lx * c - ly * s; ys[i] = b.cy + lx * s + ly * c;
    }
}
static void project(const double xs[4], const double ys[4], double ax, double ay, double *lo, double *hi) {
    *lo = 1e300; *hi = -1e300;
    for (int i = 0; i < 4; ++i) { double v = xs[i] * ax + ys[i] * ay; if (v < *lo) *lo = v; if (v > *hi) *hi = v; }
}
static int sat_obb(OBB a, OBB b) {
    double ax[4], ay[4], bx[4], by[4], axes[4][2];
    obb_corners(a, ax, ay); obb_corners(b, bx, by);
    axes[0][0] = cos(rad(a.deg)); axes[0][1] = sin(rad(a.deg)); axes[1][0] = -axes[0][1]; axes[1][1] = axes[0][0];
    axes[2][0] = cos(rad(b.deg)); axes[2][1] = sin(rad(b.deg)); axes[3][0] = -axes[2][1]; axes[3][1] = axes[2][0];
    int hit = 1;
    for (int k = 0; k < 4; ++k) {
        double l1, h1, l2, h2;
        project(ax, ay, axes[k][0], axes[k][1], &l1, &h1);
        project(bx, by, axes[k][0], axes[k][1], &l2, &h2);
        int ov = !(h1 < l2 || h2 < l1);
        printf("  axis%d A=[", k + 1); pf(l1); printf(","); pf(h1); printf("] B=["); pf(l2); printf(","); pf(h2);
        printf("] %s\n", ov ? "overlap" : "separated");
        hit = hit && ov;
    }
    return hit;
}

int main(void) {
    /* ---------- A ---------- */
    printf("[A] #226 rigid2d rot=35deg t=(0.6,-0.4)\n");
    const double px[4] = {0.0, 1.2, 1.2, 0.0}, py[4] = {0.0, 0.0, 0.8, 0.8};
    double ang = 35.0, tx = 0.6, ty = -0.4, ox[4], oy[4];
    for (int i = 0; i < 4; ++i) {
        rigid2d(px[i], py[i], ang, tx, ty, &ox[i], &oy[i]);
        printf("P%d ", i); p2(px[i], py[i]); printf(" -> "); p2(ox[i], oy[i]); printf("\n");
    }
    double c = cos(rad(ang)), s = sin(rad(ang));
    printf("cos35="); pf(c); printf(" sin35="); pf(s); printf(" det="); pf(c * c - (-s) * s); printf(" area=0.9600\n");
    printf("translate-then-rotate (wrong):");
    for (int i = 0; i < 4; ++i) { double wx, wy; rigid2d(px[i] + tx, py[i] + ty, ang, 0.0, 0.0, &wx, &wy); printf(" "); p2(wx, wy); }
    printf("\n");
    double ia, itx, ity; inv2d(ang, tx, ty, &ia, &itx, &ity);
    printf("inv2d: rot="); pf(ia); printf("deg t'="); p2(itx, ity); printf("\n");
    printf("inv2d applied:");
    for (int i = 0; i < 4; ++i) { double bx, by; rigid2d(ox[i], oy[i], ia, itx, ity, &bx, &by); printf(" "); p2(bx, by); }
    printf("\n");
    double vx = ox[1] - tx, vy = oy[1] - ty;
    double by_atan2 = deg(atan2(vy, vx)) - deg(atan2(py[1], px[1]));
    double by_acos = deg(acos((px[1] * vx + py[1] * vy) / (hypot(px[1], py[1]) * hypot(vx, vy))));
    printf("recover_angle from P1'="); p2(ox[1], oy[1]); printf(": atan2="); pf(by_atan2); printf("deg acos="); pf(by_acos);
    printf("deg wrap(-325)="); pf(wrap(-325.0)); printf("deg\n");

    /* ---------- B ---------- */
    printf("[B] #258 chain T_AC = T_AB * T_BC\n");
    T4 T_AB = homog_z(60.0, 0.5, 0.2, 0.0), T_BC = homog_x(90.0, 0.0, 0.4, 0.3);
    const double p_C[3] = {0.2, 0.1, 0.5};
    double p_B[3], p_A[3], q[3];
    T4 T_AC = mul(T_AB, T_BC);
    printf("t_AC = "); p3(T_AC.m[0][3], T_AC.m[1][3], T_AC.m[2][3]); printf("\n");
    printf("naive sum (wrong) = "); p3(0.5 + 0.0, 0.2 + 0.4, 0.0 + 0.3); printf("\n");
    apply(T_BC, p_C, p_B); printf("p_B = "); p3(p_B[0], p_B[1], p_B[2]); printf("\n");
    apply(T_AC, p_C, p_A); printf("p_A via T_AC = "); p3(p_A[0], p_A[1], p_A[2]); printf("\n");
    apply(T_AB, p_B, q); printf("p_A via two steps = "); p3(q[0], q[1], q[2]); printf("\n");
    apply(mul(T_BC, T_AB), p_C, q); printf("reversed order T_BC*T_AB (wrong) = "); p3(q[0], q[1], q[2]); printf("\n");
    apply(inv4(T_AC), p_A, q); printf("inv4(T_AC) p_A = "); p3(q[0], q[1], q[2]); printf("\n");

    /* ---------- C ---------- */
    printf("[C] #221 fk links=(1.4,0.9) angles=(25,50)\n");
    const double l2[2] = {1.4, 0.9}, a2[2] = {25.0, 50.0};
    double jx[4], jy[4];
    fk(l2, a2, 2, jx, jy);
    for (int i = 0; i < 3; ++i) { printf("j%d ", i); p2(jx[i], jy[i]); printf("\n"); }
    printf("cumulative = 25.0000/75.0000 reach = "); pf(hypot(jx[2], jy[2])); printf("\n");
    printf("absolute-angle mistake (wrong) = "); p2(jx[1] + 0.9 * cos(rad(50.0)), jy[1] + 0.9 * sin(rad(50.0))); printf("\n");
    printf("[C] #220 fk links=(1.2,0.9,0.6) angles=(20,35,-25)\n");
    const double l3[3] = {1.2, 0.9, 0.6}, a3[3] = {20.0, 35.0, -25.0};
    double kx[4], ky[4];
    fk(l3, a3, 3, kx, ky);
    for (int i = 0; i < 4; ++i) { printf("j%d ", i); p2(kx[i], ky[i]); printf("\n"); }
    printf("cumulative = 20.0000/55.0000/30.0000\n");

    /* ---------- D ---------- */
    printf("[D] #237 dh chain a=(1.4,0.9) theta=(25,50)\n");
    T4 A1 = dh(1.4, 0.0, 0.0, 25.0), A2 = dh(0.9, 0.0, 0.0, 50.0);
    printf("A1 R=["); pf(A1.m[0][0]); printf(","); pf(A1.m[0][1]); printf(";"); pf(A1.m[1][0]); printf(","); pf(A1.m[1][1]); printf("] t="); p2(A1.m[0][3], A1.m[1][3]); printf("\n");
    printf("A2 R=["); pf(A2.m[0][0]); printf(","); pf(A2.m[0][1]); printf(";"); pf(A2.m[1][0]); printf(","); pf(A2.m[1][1]); printf("] t="); p2(A2.m[0][3], A2.m[1][3]); printf("\n");
    T4 T = mul(A1, A2);
    printf("T=A1*A2 -> "); p2(T.m[0][3], T.m[1][3]); printf(" yaw=atan2(T21,T11)="); pf(deg(atan2(T.m[1][0], T.m[0][0]))); printf("deg\n");
    T4 T237 = mul(dh(2.0, 0.0, 0.0, deg(0.5236)), dh(1.5, 0.0, 0.0, deg(0.7854)));
    printf("original #237 a=(2.0,1.5) theta=(0.5236,0.7854)rad -> "); p2(T237.m[0][3], T237.m[1][3]); printf("\n");

    /* ---------- E ---------- */
    printf("[E] #223 sweep theta1=25 theta2=0..90 step 15\n");
    int n = 0;
    for (int a = 0; a <= 90; a += 15) {
        double sa[2] = {25.0, (double)a}, sx[3], sy[3];
        fk(l2, sa, 2, sx, sy);
        printf("%02d ", a); p2(sx[2], sy[2]); printf(" r_from_j1="); pf(hypot(sx[2] - jx[1], sy[2] - jy[1]));
        printf(" dist="); pf(hypot(sx[2], sy[2])); printf("\n");
        ++n;
    }
    printf("points = %d center j1 = ", n); p2(jx[1], jy[1]); printf(" annulus inner=0.5000 outer=2.3000\n");

    /* ---------- #512 ---------- */
    printf("[#512] SAT obb A=(0,0,0.6,0.4,0deg) B=(1.1,0.35,0.5,0.3,30deg)\n");
    OBB A = {0.0, 0.0, 0.6, 0.4, 0.0};
    const double cxs[2] = {1.1, 1.45};
    for (int k = 0; k < 2; ++k) {
        OBB B = {cxs[k], 0.35, 0.5, 0.3, 30.0};
        double cx[4], cy[4]; obb_corners(B, cx, cy);
        printf("B cx="); pf(cxs[k]); printf(" corners:");
        for (int i = 0; i < 4; ++i) { printf(" "); p2(cx[i], cy[i]); }
        printf("\n");
        printf("  collide = %s\n", sat_obb(A, B) ? "True" : "False");
    }

    /* ---------- #558 ---------- */
    printf("[#558] circular mean headings=(10,350,20)\n");
    const double h[3] = {10.0, 350.0, 20.0};
    printf("simple mean (wrong) = "); pf((h[0] + h[1] + h[2]) / 3.0); printf("\n");
    printf("circular mean = "); pf(circular_mean(h, 3)); printf("deg\n");
    return 0;
}
