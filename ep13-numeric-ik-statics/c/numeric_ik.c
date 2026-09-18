/* EP.13 수치 역기구학 — 뉴턴-랩슨 · DLS · 의사역행렬 · 널공간 (C, 표준 라이브러리만)
 * 빌드: gcc c/numeric_ik.c -o numeric_ik.exe -lm
 * 실행: ./numeric_ik.exe
 * 파이썬 numeric_ik.py 와 같은 값을 출력한다. */
#include <stdio.h>
#include <math.h>

#define TOL 1e-6
#define ITMAX 200
#define EPS_SING 1e-12

static void fk2(double l1, double l2, double t1, double t2, double *x, double *y) {
    *x = l1 * cos(t1) + l2 * cos(t1 + t2);
    *y = l1 * sin(t1) + l2 * sin(t1 + t2);
}

static void jacobian(double l1, double l2, double t1, double t2, double J[2][2]) {
    double s1 = sin(t1), c1 = cos(t1), s12 = sin(t1 + t2), c12 = cos(t1 + t2);
    J[0][0] = -l1 * s1 - l2 * s12; J[0][1] = -l2 * s12;
    J[1][0] =  l1 * c1 + l2 * c12; J[1][1] =  l2 * c12;
}

static double wrap(double a) {
    double w = fmod(a + M_PI, 2.0 * M_PI);
    if (w < 0) w += 2.0 * M_PI;
    return w - M_PI;
}

/* lam < 0 이면 무감쇠 뉴턴, lam >= 0 이면 DLS. 반환값: 갱신 횟수(음수면 특이 자세 중단). */
static int solve(double goal_x, double goal_y, double t1, double t2,
                 double l1, double l2, double step, double lam,
                 double *out1, double *out2, double *max_err, int verbose) {
    double J[2][2], err = 0.0, big = 0.0;
    int it;
    for (it = 0; it < ITMAX; ++it) {
        double px, py, ex, ey;
        fk2(l1, l2, t1, t2, &px, &py);
        ex = goal_x - px; ey = goal_y - py;
        err = hypot(ex, ey);
        if (err > big) big = err;
        if (verbose && it < 8) printf("    it %d  err %.6g\n", it, err);
        if (err < TOL) break;
        jacobian(l1, l2, t1, t2, J);
        if (lam < 0.0) {
            double d = J[0][0] * J[1][1] - J[0][1] * J[1][0];
            if (fabs(d) < EPS_SING) { *out1 = t1; *out2 = t2; *max_err = big; return -1; }
            t1 += step * ( J[1][1] * ex - J[0][1] * ey) / d;
            t2 += step * (-J[1][0] * ex + J[0][0] * ey) / d;
        } else {
            double a11 = J[0][0] * J[0][0] + J[0][1] * J[0][1] + lam * lam;
            double a12 = J[0][0] * J[1][0] + J[0][1] * J[1][1];
            double a22 = J[1][0] * J[1][0] + J[1][1] * J[1][1] + lam * lam;
            double det = a11 * a22 - a12 * a12;
            double u = ( a22 * ex - a12 * ey) / det;
            double v = (-a12 * ex + a11 * ey) / det;
            t1 += J[0][0] * u + J[1][0] * v;
            t2 += J[0][1] * u + J[1][1] * v;
        }
    }
    *out1 = t1; *out2 = t2; *max_err = big;
    return it;
}

static void jacobian3(const double a[3], const double th[3], double J[2][3]) {
    double cum[3];
    int k, i;
    cum[0] = th[0]; cum[1] = th[0] + th[1]; cum[2] = th[0] + th[1] + th[2];
    for (k = 0; k < 3; ++k) {
        J[0][k] = 0.0; J[1][k] = 0.0;
        for (i = k; i < 3; ++i) { J[0][k] -= a[i] * sin(cum[i]); J[1][k] += a[i] * cos(cum[i]); }
    }
}

static void pinv_vel(double J[2][3], double xdx, double xdy, double q[3]) {
    double b11 = 0, b12 = 0, b22 = 0, det, u, v;
    int k;
    for (k = 0; k < 3; ++k) { b11 += J[0][k] * J[0][k]; b12 += J[0][k] * J[1][k]; b22 += J[1][k] * J[1][k]; }
    det = b11 * b22 - b12 * b12;
    u = ( b22 * xdx - b12 * xdy) / det;
    v = (-b12 * xdx + b11 * xdy) / det;
    for (k = 0; k < 3; ++k) q[k] = J[0][k] * u + J[1][k] * v;
}

static void null_dir(double J[2][3], double n[3]) {
    double s;
    n[0] = J[0][1] * J[1][2] - J[0][2] * J[1][1];
    n[1] = J[0][2] * J[1][0] - J[0][0] * J[1][2];
    n[2] = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    s = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    n[0] /= s; n[1] /= s; n[2] /= s;
}

int main(void) {
    double t1, t2, big, x, y;
    int n, k;
    const double steps[4] = {1.0, 0.7, 0.5, 0.2};
    const double lams[5] = {0.01, 0.05, 0.1, 0.3, 1.0};
    double a3[3] = {2.0, 1.5, 1.0}, th3[3] = {0.5, 0.5, 0.5}, J3[2][3], q[3], nv[3];

    printf("== #253 Newton-Raphson (goal 2.5,1.5 / init 0.5,0.5 / step 1.0) ==\n");
    n = solve(2.5, 1.5, 0.5, 0.5, 2.0, 1.5, 1.0, -1.0, &t1, &t2, &big, 1);
    fk2(2.0, 1.5, t1, t2, &x, &y);
    printf("  theta (%.4f, %.4f)  updates %d  FK (%.4f, %.4f)\n", t1, t2, n, x, y);

    printf("== step sweep ==\n");
    for (k = 0; k < 4; ++k) {
        n = solve(2.5, 1.5, 0.5, 0.5, 2.0, 1.5, steps[k], -1.0, &t1, &t2, &big, 0);
        printf("  step %.1f -> %d updates\n", steps[k], n);
    }

    printf("== singular start, undamped (goal 3.4,0.15 / init 0,0) ==\n");
    n = solve(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, 1.0, -1.0, &t1, &t2, &big, 0);
    printf("  status %s\n", n < 0 ? "singular (stopped at first division)" : "converged");
    printf("== nudged start 0.001 — blow-up ==\n");
    n = solve(3.4, 0.15, 0.0, 0.001, 2.0, 1.5, 1.0, -1.0, &t1, &t2, &big, 0);
    fk2(2.0, 1.5, t1, t2, &x, &y);
    printf("  theta (%.4f, %.4f) rad = (%.1f, %.1f) deg  max err %.4f\n",
           t1, t2, t1 * 180.0 / M_PI, t2 * 180.0 / M_PI, big);
    printf("  FK (%.4f, %.4f)  wrapped (%.4f, %.4f)\n", x, y, wrap(t1), wrap(t2));

    printf("== DLS lam 0.1, same singular start ==\n");
    n = solve(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, 1.0, 0.1, &t1, &t2, &big, 1);
    fk2(2.0, 1.5, t1, t2, &x, &y);
    printf("  theta (%.4f, %.4f)  updates %d  FK (%.4f, %.4f)\n", t1, t2, n, x, y);

    printf("== lambda sweep ==\n");
    for (k = 0; k < 5; ++k) {
        n = solve(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, 1.0, lams[k], &t1, &t2, &big, 0);
        printf("  lam %.2f -> %3d updates   max err %.4f\n", lams[k], n, big);
    }

    printf("== 3-link pseudoinverse / null space ==\n");
    jacobian3(a3, th3, J3);
    printf("  J row0 %.4f %.4f %.4f\n", J3[0][0], J3[0][1], J3[0][2]);
    printf("  J row1 %.4f %.4f %.4f\n", J3[1][0], J3[1][1], J3[1][2]);
    pinv_vel(J3, 0.0, 1.0, q);
    null_dir(J3, nv);
    printf("  qdot %.4f %.4f %.4f  norm %.4f\n", q[0], q[1], q[2],
           sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]));
    printf("  n    %.4f %.4f %.4f\n", nv[0], nv[1], nv[2]);
    for (k = 0; k < 3; ++k) {
        double alpha = 0.5 * k, qq[3], vx = 0, vy = 0, nr;
        int i;
        for (i = 0; i < 3; ++i) { qq[i] = q[i] + alpha * nv[i]; vx += J3[0][i] * qq[i]; vy += J3[1][i] * qq[i]; }
        nr = sqrt(qq[0] * qq[0] + qq[1] * qq[1] + qq[2] * qq[2]);
        printf("  alpha %.1f  norm %.4f  tip (%.4f, %.4f)\n", alpha, nr, vx, vy);
    }
    return 0;
}
