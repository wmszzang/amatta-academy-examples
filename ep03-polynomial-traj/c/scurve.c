// S-커브(저크 제한) 속도 프로파일 — C 버전. scurve.csv 를 만든다.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl scurve.c
//     scurve.exe
// 컴파일·실행 (gcc / MinGW):
//     gcc scurve.c -o scurve -lm
//     ./scurve                   (Windows: scurve.exe)
//
// 예제 조건(영상과 동일): 목표 90도, 최고 속도 60, 최고 가속도 120, 저크 한계 480
//   tj = A/J = 0.25초 → 가속 구간 0.75초 · 등속 0.75초 · 감속 0.75초 = 총 2.25초
// 핵심: 7구간마다 저크를 +J / 0 / -J 로 두고, 가속도 → 속도 → 위치를 차례로 누적(적분).
#include <math.h>
#include <stdio.h>

int main(void) {
    const double L = 90.0, V = 60.0, J = 480.0, DT = 0.001;
    double A = 120.0;
    double tj = A / J;                       /* 저크 구간 길이 */
    if (V / A < tj) {                        /* 최고 가속도에 못 미치는 짧은 경우 */
        tj = sqrt(V / J);
        A = J * tj;
    }
    double ta = V / A + tj;                  /* 가속 구간 전체 */
    double da = V * ta / 2;                  /* 가속 구간 거리 */
    double tc = (L - 2 * da) / V;            /* 등속 시간 */
    double dur[7] = {tj, ta - 2 * tj, tj, tc, tj, ta - 2 * tj, tj};
    double jerk[7] = {+J, 0.0, -J, 0.0, -J, 0.0, +J};

    FILE *f = fopen("scurve.csv", "w");
    if (f == NULL) { printf("cannot open scurve.csv\n"); return 1; }
    fprintf(f, "t,x,v,a,j\n");
    double t = 0.0, x = 0.0, v = 0.0, a = 0.0;
    fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.1f\n", t, x, v, a, J);
    for (int k = 0; k < 7; ++k) {
        int n = (int)(dur[k] / DT + 0.5);
        for (int i = 0; i < n; ++i) {
            double j = jerk[k];
            x += v * DT + a * DT * DT / 2 + j * DT * DT * DT / 6;   /* 위치 */
            v += a * DT + j * DT * DT / 2;                          /* 속도 */
            a += j * DT;                                            /* 가속도 */
            t += DT;
            fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.1f\n", t, x, v, a, j);
        }
    }
    fclose(f);
    printf("total time = %.2f s, final position = %.2f deg\n", t, x);
    printf("saved: scurve.csv\n");
    return 0;
}
