/* EP.14 역기구학·자코비안 끝내기 — C99, 라이브러리 없이 배열만 쓴다(시험장 재현용).
 * 빌드: gcc -std=c99 kin14.c -o kin14.exe -lm
 * 파이썬·C++ 과 같은 입력에서 같은 출력을 내는지 대조하는 것이 목적이다. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TOL 1e-6
#define GRAV 9.81
#define PI 3.14159265358979323846

typedef struct { double x, y; } Vec2;

static double wrap_angle(double a) { return atan2(sin(a), cos(a)); }

static Vec2 fk2(double a1, double a2, double t1, double t2) {
    Vec2 p;
    p.x = a1 * cos(t1) + a2 * cos(t1 + t2);
    p.y = a1 * sin(t1) + a2 * sin(t1 + t2);
    return p;
}

/* J[0][*] = 손끝 가로속도 성분, J[1][*] = 세로속도 성분. 열은 관절. */
static void jac(double a1, double a2, double t1, double t2, double J[2][2]) {
    double s1 = sin(t1), c1 = cos(t1), s12 = sin(t1 + t2), c12 = cos(t1 + t2);
    J[0][0] = -a1 * s1 - a2 * s12;  J[0][1] = -a2 * s12;
    J[1][0] =  a1 * c1 + a2 * c12;  J[1][1] =  a2 * c12;
}

static double det2(double J[2][2]) { return J[0][0] * J[1][1] - J[0][1] * J[1][0]; }

static Vec2 velocity(double J[2][2], Vec2 dq) {      /* v = J qdot */
    Vec2 v;
    v.x = J[0][0] * dq.x + J[0][1] * dq.y;
    v.y = J[1][0] * dq.x + J[1][1] * dq.y;
    return v;
}

static Vec2 torque(double J[2][2], Vec2 F) {         /* tau = J^T F */
    Vec2 t;
    t.x = J[0][0] * F.x + J[1][0] * F.y;
    t.y = J[0][1] * F.x + J[1][1] * F.y;
    return t;
}

static void svd2(double J[2][2], double *smax, double *smin) {
    double a = J[0][0] * J[0][0] + J[1][0] * J[1][0];
    double b = J[0][0] * J[0][1] + J[1][0] * J[1][1];
    double d = J[0][1] * J[0][1] + J[1][1] * J[1][1];
    double tr = a + d, dt = a * d - b * b;
    double disc = sqrt(fmax(0.0, tr * tr / 4 - dt));
    *smax = sqrt(fmax(0.0, tr / 2 + disc));
    *smin = sqrt(fmax(0.0, tr / 2 - disc));
}

static int ik2(double a1, double a2, double x, double y, int elbow_up, Vec2 *out) {
    double r2 = x * x + y * y, r = sqrt(r2), c2, s2;
    if (r < fabs(a1 - a2) - 1e-12 || r > a1 + a2 + 1e-12) return 0;
    c2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;
    s2 = sqrt(fmax(0.0, 1 - c2 * c2));
    if (elbow_up) s2 = -s2;
    out->y = wrap_angle(atan2(s2, c2));
    out->x = wrap_angle(atan2(y, x) - atan2(a2 * s2, a1 + a2 * c2));
    return 1;
}

static int cmp_pt(const void *pa, const void *pb) {
    const Vec2 *a = (const Vec2 *)pa, *b = (const Vec2 *)pb;
    if (a->x < b->x) return -1;
    if (a->x > b->x) return 1;
    if (a->y < b->y) return -1;
    if (a->y > b->y) return 1;
    return 0;
}

static double cross_oab(Vec2 o, Vec2 a, Vec2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

/* 모노톤 체인 — 정렬한 점을 아래 체인·위 체인으로 두 번 훑는다. */
static int hull(Vec2 *P, int n, Vec2 *out) {
    int k = 0, i, start = 2;
    qsort(P, n, sizeof(Vec2), cmp_pt);
    for (i = 0; i < n; i++) {
        while (k >= start && cross_oab(out[k - 2], out[k - 1], P[i]) <= 0) k--;
        out[k++] = P[i];
    }
    k--;
    start = k + 2;
    for (i = n - 1; i >= 0; i--) {
        while (k >= start && cross_oab(out[k - 2], out[k - 1], P[i]) <= 0) k--;
        out[k++] = P[i];
    }
    return k - 1;
}

static double shoelace(Vec2 *P, int n) {
    double s = 0;
    int i;
    for (i = 0; i < n; i++) s += P[i].x * P[(i + 1) % n].y - P[(i + 1) % n].x * P[i].y;
    return fabs(s) / 2;
}

static int in_poly(Vec2 p, Vec2 *poly, int n) {
    int inside = 0, i;
    for (i = 0; i < n; i++) {
        Vec2 a = poly[i], b = poly[(i + 1) % n];
        if ((a.y > p.y) != (b.y > p.y)) {
            double xc = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (p.x < xc) inside = !inside;
        }
    }
    return inside;
}

/* 반환값 = 오차 검사 횟수. 각도 갱신 횟수는 그보다 1 적다. */
static int newton_dls(double gx, double gy, double *t1, double *t2,
                      double a1, double a2, double lam, int *ok) {
    int it;
    *ok = 0;
    for (it = 1; it <= 200; it++) {
        Vec2 p = fk2(a1, a2, *t1, *t2);
        double ex = gx - p.x, ey = gy - p.y;
        double J[2][2], d;
        if (sqrt(ex * ex + ey * ey) < TOL) { *ok = 1; return it; }
        jac(a1, a2, *t1, *t2, J);
        if (lam > 0) {
            double A = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lam * lam;
            double B = J[0][0] * J[0][1] + J[1][0] * J[1][1];
            double C = J[0][1] * J[0][1] + J[1][1] * J[1][1] + lam * lam;
            double g0 = J[0][0] * ex + J[1][0] * ey;
            double g1 = J[0][1] * ex + J[1][1] * ey;
            double D = A * C - B * B;
            if (D == 0) return it;
            *t1 += (C * g0 - B * g1) / D;
            *t2 += (-B * g0 + A * g1) / D;
        } else {
            d = det2(J);
            if (d == 0) return it;
            *t1 += (J[1][1] * ex - J[0][1] * ey) / d;
            *t2 += (-J[1][0] * ex + J[0][0] * ey) / d;
        }
    }
    return 200;
}

int main(void) {
    const double a1 = 2.0, a2 = 1.5;
    double J[2][2], smax, smin, tg1, tg2;
    Vec2 s, f, dq, F, t, wrongt;
    int i, up;

    printf("=== (1) 해석 IK ===\n");
    ik2(1.0, 1.0, 1.5, 1.0, 0, &s);
    f = fk2(1.0, 1.0, s.x, s.y);
    printf("#222 th=(%.4f, %.4f) deg  FK=(%.4f, %.4f)\n",
           s.x * 180 / PI, s.y * 180 / PI, f.x, f.y);
    for (up = 0; up < 2; up++) {
        ik2(a1, a2, 2.5, 1.5, up, &s);
        f = fk2(a1, a2, s.x, s.y);
        printf("#242 %-4s th=(%.4f, %.4f) rad  FK=(%.4f, %.4f)\n",
               up ? "up" : "down", s.x, s.y, f.x, f.y);
    }
    {
        double tx[2] = {3.0, 0.3}, ty[2] = {2.0, 0.2};
        for (i = 0; i < 2; i++) {
            double r = sqrt(tx[i] * tx[i] + ty[i] * ty[i]);
            double c2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2);
            printf("도달 판정 (%.1f, %.1f) r=%.4f cos(th2)=%.4f -> %s\n", tx[i], ty[i], r, c2,
                   (r >= fabs(a1 - a2) && r <= a1 + a2) ? "REACHABLE" : "UNREACHABLE");
        }
    }
    printf("#477 6.0 -> wrap %.4f,  7.3208 -> wrap %.4f\n", wrap_angle(6.0), wrap_angle(7.3208));

    printf("\n=== (2) 작업공간 ===\n");
    ik2(a1, a2, 2.5, 1.5, 0, &s);
    jac(a1, a2, s.x, s.y, J);
    svd2(J, &smax, &smin);
    printf("#257 r=%.4f REACHABLE  w=%.4f  smax=%.4f  smin=%.4f  kappa=%.4f\n",
           sqrt(2.5 * 2.5 + 1.5 * 1.5), fabs(det2(J)), smax, smin, smax / smin);
    printf("     환형 넓이 = %.4f m^2\n",
           PI * ((a1 + a2) * (a1 + a2) - (a1 - a2) * (a1 - a2)));
    {
        static Vec2 pts[625], ch[1300];
        Vec2 fence[5] = {{-1.8, -0.4}, {3.8, -0.4}, {3.8, 2.4}, {1.0, 3.8}, {-1.8, 2.4}};
        int n = 0, m, outside = 0, j;
        for (i = 0; i < 25; i++)
            for (j = 0; j < 25; j++)
                pts[n++] = fk2(a1, a2, (90.0 * i / 24) * PI / 180, (120.0 * j / 24) * PI / 180);
        for (i = 0; i < n; i++)
            if (!in_poly(pts[i], fence, 5)) outside++;
        m = hull(pts, n, ch);
        printf("#497 스윕 %d점 -> 껍질 %d정점\n", n, m);
        printf("#287 껍질 넓이 = %.4f, 펜스 넓이 = %.4f, 펜스 밖 = %d/%d\n",
               shoelace(ch, m), shoelace(fence, 5), outside, n);
    }

    printf("\n=== (3) 속도·힘 ===\n");
    jac(a1, a2, 0.5, 0.5, J);
    printf("J = [[%.4f, %.4f], [%.4f, %.4f]]  det J = %.4f\n",
           J[0][0], J[0][1], J[1][0], J[1][1], det2(J));
    dq.x = 0.4; dq.y = -0.3;
    s = velocity(J, dq);
    printf("#239 v = (%.4f, %.4f) m/s  |v| = %.4f\n", s.x, s.y, sqrt(s.x * s.x + s.y * s.y));
    F.x = 3.0; F.y = -1.0;
    t = torque(J, F);
    wrongt = velocity(J, F);
    printf("#260 tau = J^T F = (%.4f, %.4f) N.m\n", t.x, t.y);
    printf("     전치 누락 J F = (%.4f, %.4f), 차이 크기 = %.4f\n", wrongt.x, wrongt.y,
           sqrt((t.x - wrongt.x) * (t.x - wrongt.x) + (t.y - wrongt.y) * (t.y - wrongt.y)));
    printf("     가상일 F.v = %.6f W, tau.qdot = %.6f W\n",
           F.x * s.x + F.y * s.y, t.x * dq.x + t.y * dq.y);
    {
        double t2s[5] = {0.5, 0.2, 0.05, 0.01, 0.001};
        for (i = 0; i < 5; i++) {
            double Ji[2][2], d, q1, q2;
            jac(a1, a2, 0.5, t2s[i], Ji);
            d = det2(Ji);
            q1 = (Ji[1][1] * 0.0 - Ji[0][1] * 0.2) / d;
            q2 = (-Ji[1][0] * 0.0 + Ji[0][0] * 0.2) / d;
            printf("     th2=%.3f  det J=%.6f  |qdot|=%.4f rad/s\n",
                   t2s[i], d, sqrt(q1 * q1 + q2 * q2));
        }
    }
    tg2 = (2.0 * a2 / 2) * GRAV * cos(1.0);
    tg1 = (3.0 * a1 / 2 + 2.0 * a1) * GRAV * cos(0.5) + tg2;
    printf("중력 보상 tau_g = (%.4f, %.4f) N.m, 지령 = (%.4f, %.4f) N.m\n",
           tg1, tg2, tg1 + t.x, tg2 + t.y);

    printf("\n=== (4) 수치 IK ===\n");
    {
        double st1[4] = {0.5, 0.0, 0.0, 0.1}, st2[4] = {0.5, 0.0, 0.0, 0.05};
        double lams[4] = {0.0, 0.0, 0.05, 0.0};
        for (i = 0; i < 4; i++) {
            double t1 = st1[i], t2 = st2[i];
            int ok, checks = newton_dls(2.5, 1.5, &t1, &t2, a1, a2, lams[i], &ok);
            if (ok)
                printf("lam=%.2f 시작 (%.2f, %.2f)  검사 %2d회 / 갱신 %2d회  raw=(%.4f, %.4f) wrap=(%.4f, %.4f)\n",
                       lams[i], st1[i], st2[i], checks, checks - 1, t1, t2,
                       wrap_angle(t1), wrap_angle(t2));
            else
                printf("lam=%.2f 시작 (%.2f, %.2f)  검사 %2d회 / 갱신 %2d회  실패: det J = 0\n",
                       lams[i], st1[i], st2[i], checks, checks - 1);
        }
    }

    printf("\n=== (5) 안정성 ===\n");
    {
        const char *names[2] = {"folded", "extended"};
        double p1[2] = {0.5, 0.1}, p2[2] = {0.5, 0.05};
        for (i = 0; i < 2; i++) {
            double ex = a1 * cos(p1[i]), ey = a1 * sin(p1[i]);
            Vec2 tip = fk2(a1, a2, p1[i], p2[i]);
            double total = 36.0;
            double cx = (3.0 * ex / 2 + 2.0 * (ex + tip.x) / 2 + 1.0 * tip.x) / total;
            double cy = (3.0 * ey / 2 + 2.0 * (ey + tip.y) / 2 + 1.0 * tip.y) / total;
            double margin = fmin(0.30 - fabs(cx), 0.30 - fabs(cy));
            printf("#484 %-9s CoM 투영 (%.4f, %.4f)  여유 %+.4f m  %s\n",
                   names[i], cx, cy, margin, margin >= 0 ? "안정" : "전도");
        }
    }
    return 0;
}
