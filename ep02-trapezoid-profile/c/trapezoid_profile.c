/* 사다리꼴 속도 프로파일 운동계획 — C 예제 답안.
 *
 * Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.2
 *
 * 컴파일·실행 (gcc / MinGW):
 *     gcc trapezoid_profile.c -o trapezoid -lm
 *     ./trapezoid            (Windows: trapezoid.exe)
 *
 * 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
 *     cl trapezoid_profile.c
 *     trapezoid_profile.exe
 *
 * → 콘솔 요약 출력 + profile.csv(시간,거리,속도) 저장.
 *   그래프는 python 폴더의 visualize.py로 그립니다(언어 무관, README 참고).
 */
#include <math.h>
#include <stdio.h>

int main(void) {
    /* 영상 예제 그대로: 목표 180도, 최고 속도 초당 90도, 가속도 매초 초당 180도 */
    double L = 180.0, V = 90.0, A = 180.0, dt = 0.01;

    double D = V * V / (2.0 * A);          /* 가속에 필요한 거리 */
    if (2.0 * D >= L)                      /* 목표가 짧으면: 삼각형 */
        V = sqrt(A * L);
    double t1 = V / A;                     /* 가속이 끝나는 시각 */
    double rest = L - V * V / A;           /* 등속이 맡을 거리 */
    double tc = (rest > 0.0 ? rest : 0.0) / V;
    double T = 2.0 * t1 + tc;              /* 총 시간 */

    FILE *f = fopen("profile.csv", "w");
    if (f == NULL) {
        printf("profile.csv 파일을 열 수 없습니다.\n");
        return 1;
    }
    fprintf(f, "t,s,v\n");
    double s = 0.0;
    for (double t = 0.0; t <= T; t += dt) {
        double v;
        if (t < t1)            v = A * t;
        else if (t < t1 + tc)  v = V;
        else {
            v = V - A * (t - t1 - tc);
            if (v < 0.0) v = 0.0;
        }
        s += v * dt;
        fprintf(f, "%.3f,%.4f,%.4f\n", t, s, v);
    }
    fclose(f);
    printf("총 시간 = %.2f초, 최종 거리 = %.1f도  →  profile.csv 저장 완료\n", T, s);
    return 0;
}
