// 3차·5차 다항식 관절 궤적 — C 버전. cubic.csv, quintic.csv 를 만든다.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl polynomial_traj.c
//     polynomial_traj.exe
// 컴파일·실행 (gcc / MinGW):
//     gcc polynomial_traj.c -o polynomial_traj -lm
//     ./polynomial_traj          (Windows: polynomial_traj.exe)
//
// 예제 조건(영상과 동일): 0도 → 90도, 2초. 시간 간격 0.01초.
//   3차: a2 = 3·d/T² = 67.5, a3 = -2·d/T³ = -22.5
//   5차: q = q0 + d·(10s³ - 15s⁴ + 6s⁵), s = t/T
#include <stdio.h>

int main(void) {
    const double Q0 = 0.0, QF = 90.0, T = 2.0, DT = 0.01;
    const double d = QF - Q0;
    const double a2 = 3 * d / (T * T), a3 = -2 * d / (T * T * T);
    FILE *fc = fopen("cubic.csv", "w");
    FILE *fq = fopen("quintic.csv", "w");
    if (fc == NULL || fq == NULL) { printf("cannot open output files\n"); return 1; }
    fprintf(fc, "t,q,qd,qdd\n");
    fprintf(fq, "t,q,qd,qdd\n");
    for (double t = 0.0; t <= T + 1e-9; t += DT) {
        /* 3차: 각도 → 각속도 → 각가속도 (미분 규칙: 지수를 내려 곱하고 지수를 하나 줄인다) */
        double q   = Q0 + a2 * t * t + a3 * t * t * t;
        double qd  = 2 * a2 * t + 3 * a3 * t * t;
        double qdd = 2 * a2 + 6 * a3 * t;
        fprintf(fc, "%.3f,%.4f,%.4f,%.4f\n", t, q, qd, qdd);

        /* 5차: 정규화 시간 s = t/T 로 10·15·6 공식 */
        double s = t / T, s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
        double p   = Q0 + d * (10 * s3 - 15 * s4 + 6 * s5);
        double pd  = d * (30 * s2 - 60 * s3 + 30 * s4) / T;
        double pdd = d * (60 * s - 180 * s2 + 120 * s3) / (T * T);
        fprintf(fq, "%.3f,%.4f,%.4f,%.4f\n", t, p, pd, pdd);
    }
    fclose(fc);
    fclose(fq);
    printf("saved: cubic.csv, quintic.csv\n");
    return 0;
}
