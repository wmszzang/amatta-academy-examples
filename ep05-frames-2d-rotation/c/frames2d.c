/* 프레임 · 2D 회전 · 강체 변환 — 예제 답안 (C)
 * Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.5
 *
 * Python 판(python/frames2d.py)과 같은 계산 · 같은 출력(points.csv, angles.txt).
 * 빌드: cl frames2d.c   (Visual Studio 개발자 명령 프롬프트)   /   gcc frames2d.c -o frames2d -lm
 */
#include <math.h>
#include <stdio.h>

static const double PI = 3.14159265358979323846;   /* M_PI 는 표준이 아니라 직접 정의 */

typedef struct { double x, y; } Pt;
typedef struct { double c, s; } Rot;                 /* R(θ) = [[c, -s], [s, c]] */

static Rot rot2d(double deg) {
    double t = deg * PI / 180.0;                     /* 코드의 sin/cos 는 라디안만 받는다 */
    Rot r; r.c = cos(t); r.s = sin(t); return r;
}
static Pt apply(Rot R, Pt p) {                       /* R p : 첫째 열 × x + 둘째 열 × y */
    Pt q; q.x = R.c * p.x - R.s * p.y; q.y = R.s * p.x + R.c * p.y; return q;
}
static Pt to_base(Rot R, Pt t, Pt p) {               /* 툴 → 베이스 : p' = R p + t (회전 후 이동) */
    Pt q = apply(R, p); q.x += t.x; q.y += t.y; return q;
}
static Pt to_tool(Rot R, Pt t, Pt p) {               /* 베이스 → 툴 : 이동량 빼고 Rᵀ 곱하기 */
    double dx = p.x - t.x, dy = p.y - t.y;
    Pt q; q.x = R.c * dx + R.s * dy; q.y = -R.s * dx + R.c * dy; return q;
}
static double orth_check(Rot R, double rtr[2][2]) {  /* RᵀR 계산, 항등행렬과의 최대 오차 반환 */
    double a = R.c, b = -R.s, c = R.s, d = R.c;
    rtr[0][0] = a * a + c * c; rtr[0][1] = a * b + c * d;
    rtr[1][0] = b * a + d * c; rtr[1][1] = b * b + d * d;
    double e = fabs(rtr[0][0] - 1); if (fabs(rtr[0][1]) > e) e = fabs(rtr[0][1]);
    if (fabs(rtr[1][0]) > e) e = fabs(rtr[1][0]); if (fabs(rtr[1][1] - 1) > e) e = fabs(rtr[1][1] - 1);
    return e;
}
static double wrap(double a) {                       /* [−π, π] 로 접기 — C 의 fmod 는 부호를 유지하므로 보정 */
    double m = fmod(a + PI, 2 * PI);
    if (m < 0) m += 2 * PI;
    return m - PI;
}
static double wrap_deg(double d) {
    double m = fmod(d + 180.0, 360.0);
    if (m < 0) m += 360.0;
    return m - 180.0;
}
static double ang_err(double target, double current) { return wrap(target - current); }
static double circ_mean_deg(const double *degs, int n) {   /* atan2(Σ sin, Σ cos) — y 먼저, x 나중 */
    double s = 0, c = 0; int i;
    for (i = 0; i < n; ++i) { s += sin(degs[i] * PI / 180.0); c += cos(degs[i] * PI / 180.0); }
    return atan2(s, c) * 180.0 / PI;
}

int main(void) {
    Pt square[4] = { {0, 0}, {1, 0}, {1, 1}, {0, 1} };
    double deg = 30.0; Pt t = { 1.0, 0.5 };
    Rot R = rot2d(deg);
    Pt moved[4]; int i;
    for (i = 0; i < 4; ++i) moved[i] = to_base(R, t, square[i]);

    printf("R(30) = [[%.4f, %.4f], [%.4f, %.4f]]\n", R.c, -R.s, R.s, R.c);
    printf("square 30deg t=(1,0.5):");
    for (i = 0; i < 4; ++i) printf(" (%.4f, %.4f)", moved[i].x, moved[i].y);
    printf("\n");
    FILE *f = fopen("points.csv", "w");
    if (!f) { printf("points.csv open failed\n"); return 1; }
    fprintf(f, "x,y,xr,yr\n");
    for (i = 0; i < 4; ++i) fprintf(f, "%.4f,%.4f,%.4f,%.4f\n", square[i].x, square[i].y, moved[i].x, moved[i].y);
    fclose(f);

    Pt p10 = { 1, 0 }, p21 = { 2, 1 };
    Pt pb = to_base(R, t, p10), pt = to_tool(R, t, p21), back = to_base(R, t, pt);
    printf("tool(1,0)->base(%.4f, %.4f)\n", pb.x, pb.y);
    printf("base(2,1)->tool(%.4f, %.4f)\n", pt.x, pt.y);
    printf("->back(%.4f, %.4f)\n", back.x, back.y);
    Pt shifted = { 1 + t.x, 0 + t.y }; Pt mr = apply(R, shifted);
    printf("move-then-rotate (1,0): (%.4f, %.4f)   (rotate-then-move: (%.4f, %.4f))\n", mr.x, mr.y, moved[1].x, moved[1].y);

    double rtr[2][2]; double err = orth_check(R, rtr);
    printf("RtR = [[%.4f, %.4f], [%.4f, %.4f]]  max_err=%.1e\n", rtr[0][0], rtr[0][1], rtr[1][0], rtr[1][1], err);
    printf("det = %.4f   trace = %.4f\n", R.c * R.c + R.s * R.s, 2 * R.c);

    double a[2] = { 170, -170 }, b[3] = { 10, -10, 350 };
    char lines[6][64];
    sprintf(lines[0], "ang_err(3.0,-3.0)=%.4f", ang_err(3.0, -3.0));
    sprintf(lines[1], "ang_err(0.1,-0.1)=%.4f", ang_err(0.1, -0.1));
    sprintf(lines[2], "circ_mean([170,-170])=%.4f", circ_mean_deg(a, 2));
    sprintf(lines[3], "circ_mean([10,-10,350])=%.4f", circ_mean_deg(b, 3));
    sprintf(lines[4], "wrap(359deg)=%.4fdeg", wrap_deg(359.0));
    sprintf(lines[5], "orth_check(30)=%s", err < 1e-12 ? "identity(max_err<1e-12)" : "FAIL");
    FILE *g = fopen("angles.txt", "w");
    if (!g) { printf("angles.txt open failed\n"); return 1; }
    for (i = 0; i < 6; ++i) { printf("%s\n", lines[i]); fprintf(g, "%s\n", lines[i]); }
    fclose(g);
    printf("(naive) 3.0-(-3.0)=%.4f   mean([170,-170])=%.4f   mean([10,-10,350])=%.4f\n", 6.0, 0.0, (10 - 10 + 350) / 3.0);
    printf("saved: points.csv, angles.txt\n");
    return 0;
}
