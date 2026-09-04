/* 운동계획 끝내기 - 네 지문(사다리꼴 · 3차 · 5차 · S-커브)을 한 파일로. C 버전.
 *
 * Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.4
 *
 * 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
 *     cl motion_wrapup.c
 *     motion_wrapup.exe
 * 컴파일·실행 (gcc / MinGW):
 *     gcc motion_wrapup.c -o motion_wrapup -lm
 *     ./motion_wrapup                (Windows: motion_wrapup.exe)
 *
 * 출력: p1_trapezoid.csv (t,x,v,a) · p2_cubic.csv (t,q,qd,qdd) · p3_quintic.csv (t,q,qd,qdd) · p4_scurve.csv (t,x,v,a,j)
 */
#include <math.h>
#include <stdio.h>

/* ── 1. 사다리꼴 (삼각형 축퇴 포함) ──────────────────────────────────────── */
static void trapezoid(double D, double V, double A, double dt, const char* name) {
    if (V * V / A > D) V = sqrt(A * D);              /* 가속+감속 거리(V²/A)가 목표보다 길면 삼각형 */
    double ta = V / A;                               /* 가속 시간 */
    double tc = (D - V * V / A) / V;                 /* 등속 시간 (삼각형이면 0) */
    double T = 2 * ta + tc;
    FILE* f = fopen(name, "w");
    if (f == NULL) { printf("cannot open %s\n", name); return; }
    fprintf(f, "t,x,v,a\n");
    int n = (int)(T / dt + 0.5), i;
    double x = 0.0, v = 0.0, a = 0.0;
    for (i = 0; i <= n; ++i) {
        double t = i * dt;
        if (t < ta)           { a = A;   v = A * t;   x = A * t * t / 2; }
        else if (t < ta + tc) { a = 0.0; v = V;       x = V * ta / 2 + V * (t - ta); }
        else { double r = T - t; if (r < 0) r = 0; a = -A; v = A * r; x = D - A * r * r / 2; }
        fprintf(f, "%.2f,%.4f,%.4f,%.1f\n", t, x, v, a);        /* 열별 소수 자릿수 = Python판과 동일 */
    }
    fclose(f);
    printf("P1 trapezoid: total %.2f s, peak %.1f, final %.2f deg -> %s\n", T, V, x, name);
}

/* ── 2. 3차 다항식 (시작·끝 속도 0) ─────────────────────────────────────── */
static void cubic(double q0, double qf, double T, double dt, const char* name) {
    double d = qf - q0;                              /* 이동량 = 목표 - 시작 */
    double a2 = 3 * d / (T * T), a3 = -2 * d / (T * T * T);
    FILE* f = fopen(name, "w");
    if (f == NULL) { printf("cannot open %s\n", name); return; }
    fprintf(f, "t,q,qd,qdd\n");
    int n = (int)(T / dt + 0.5), i;
    double q = q0;
    for (i = 0; i <= n; ++i) {
        double t = i * dt;
        q = q0 + a2 * t * t + a3 * t * t * t;        /* 상수항 = 시작 각도 */
        fprintf(f, "%.1f,%.4f,%.4f,%.4f\n", t, q, 2 * a2 * t + 3 * a3 * t * t, 2 * a2 + 6 * a3 * t);
    }
    fclose(f);
    printf("P2 cubic: final %.2f deg -> %s\n", q, name);
}

/* ── 3. 5차 다항식 (시작·끝 속도·가속도 0 - 10·15·6 공식) ────────────────── */
static void quintic(double q0, double qf, double T, double dt, const char* name) {
    double d = qf - q0;
    FILE* f = fopen(name, "w");
    if (f == NULL) { printf("cannot open %s\n", name); return; }
    fprintf(f, "t,q,qd,qdd\n");
    int n = (int)(T / dt + 0.5), i;
    double q = q0;
    for (i = 0; i <= n; ++i) {
        double t = i * dt, s = t / T;                /* 진행률 0 → 1 */
        q = q0 + d * (10 * pow(s, 3) - 15 * pow(s, 4) + 6 * pow(s, 5));
        fprintf(f, "%.1f,%.4f,%.4f,%.4f\n", t, q,
                d * (30 * s * s - 60 * pow(s, 3) + 30 * pow(s, 4)) / T,
                d * (60 * s - 180 * s * s + 120 * pow(s, 3)) / (T * T));
    }
    fclose(f);
    printf("P3 quintic: final %.2f deg -> %s\n", q, name);
}

/* ── 4. S-커브 (저크 제한 7구간 - 도달 못 하는 구간은 0으로) ────────────── */
static void scurve(double L, double V, double A, double J, double dt, const char* name) {
    double tj = A / J;                                        /* 저크 구간 */
    if (V / A < tj) { tj = sqrt(V / J); A = J * tj; }         /* 확인 1: 가속 일정 구간이 없다 */
    double ta = V / A + tj, da = V * ta / 2;
    if (L < 2 * da) {                                         /* 확인 2: 등속 구간이 없다 */
        V = (-tj + sqrt(tj * tj + 4 * L / A)) * A / 2;        /*   V²/A + V·tj = L (근의 공식) */
        if (V / A < tj) { tj = cbrt(L / (2 * J)); A = J * tj; V = J * tj * tj; }
        ta = V / A + tj; da = V * ta / 2;
    }
    double tc = (L - 2 * da) / V;
    double dur[7]  = {tj, ta - 2 * tj, tj, tc, tj, ta - 2 * tj, tj};
    double jerk[7] = {+J, 0.0, -J, 0.0, -J, 0.0, +J};
    FILE* f = fopen(name, "w");
    if (f == NULL) { printf("cannot open %s\n", name); return; }
    fprintf(f, "t,x,v,a,j\n");
    double t = 0, x = 0, v = 0, a = 0;
    fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.1f\n", t, x, v, a, J);
    int k;
    for (k = 0; k < 7; ++k) {
        double t_end = t + dur[k], j = jerk[k];
        while (t < t_end - 1e-9) {
            double h = t_end - t; if (h > dt) h = dt;         /* 구간 끝은 마지막 걸음만 짧게 */
            x += v * h + a * h * h / 2 + j * h * h * h / 6;
            v += a * h + j * h * h / 2;
            a += j * h;
            t += h;
            fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.1f\n", t, x, v, a, j);
        }
    }
    fclose(f);
    printf("P4 s-curve: total %.3f s, final %.2f deg -> %s\n", t, x, name);
}

int main(void) {
    trapezoid(150, 60, 120, 0.01, "p1_trapezoid.csv");      /* 지문 1: 한계 60·120, 시간 없음 */
    cubic(30, 120, 3, 0.1, "p2_cubic.csv");                 /* 지문 2: 30→120도, 3초, 정지→정지 */
    quintic(-45, 45, 4, 0.2, "p3_quintic.csv");             /* 지문 3: -45→45도, 4초, 각가속도도 0 */
    scurve(180, 90, 180, 720, 0.005, "p4_scurve.csv");      /* 지문 4: 한계 90·180·저크 720 */
    return 0;
}
