// S-커브(저크 제한) 속도 프로파일 — C++ 버전. scurve.csv 를 만든다.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl /EHsc scurve.cpp
//     scurve.exe
// 컴파일·실행 (g++ / MinGW):
//     g++ -std=c++11 scurve.cpp -o scurve
//     ./scurve                   (Windows: scurve.exe)
//
// 예제 조건(영상과 동일): 목표 90도, 최고 속도 60, 최고 가속도 120, 저크 한계 480 → 총 2.25초
// 핵심: 7구간마다 저크를 +J / 0 / -J 로 두고, 가속도 → 속도 → 위치를 차례로 누적(적분).
#include <cmath>
#include <cstdio>
#include <vector>

struct Row { double t, x, v, a, j; };

std::vector<Row> scurve(double L, double V, double A, double J, double dt = 0.001) {
    double tj = A / J;                       // 저크 구간 길이
    if (V / A < tj) {                        // 최고 가속도에 못 미치는 짧은 경우
        tj = std::sqrt(V / J);
        A = J * tj;
    }
    double ta = V / A + tj;                  // 가속 구간 전체(저크+ · 일정 · 저크-)
    double da = V * ta / 2;                  // 가속 구간 거리
    double tc = (L - 2 * da) / V;            // 등속 시간
    double dur[7]  = {tj, ta - 2 * tj, tj, tc, tj, ta - 2 * tj, tj};
    double jerk[7] = {+J, 0.0, -J, 0.0, -J, 0.0, +J};
    std::vector<Row> rows = {{0.0, 0.0, 0.0, 0.0, +J}};
    double t = 0.0, x = 0.0, v = 0.0, a = 0.0;
    for (int k = 0; k < 7; ++k) {
        int n = static_cast<int>(dur[k] / dt + 0.5);
        for (int i = 0; i < n; ++i) {
            double j = jerk[k];
            x += v * dt + a * dt * dt / 2 + j * dt * dt * dt / 6;   // 위치
            v += a * dt + j * dt * dt / 2;                          // 속도
            a += j * dt;                                            // 가속도
            t += dt;
            rows.push_back({t, x, v, a, j});
        }
    }
    return rows;
}

int main() {
    std::vector<Row> rows = scurve(90.0, 60.0, 120.0, 480.0);
    std::FILE* f = std::fopen("scurve.csv", "w");
    if (f == nullptr) { std::printf("cannot open scurve.csv\n"); return 1; }
    std::fprintf(f, "t,x,v,a,j\n");
    for (const Row& r : rows)
        std::fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.1f\n", r.t, r.x, r.v, r.a, r.j);
    std::fclose(f);
    std::printf("total time = %.2f s, final position = %.2f deg\n", rows.back().t, rows.back().x);
    std::printf("saved: scurve.csv\n");
    return 0;
}
