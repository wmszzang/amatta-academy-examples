// 사다리꼴 속도 프로파일 운동계획 — C++ 예제 답안.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.2
//
// 컴파일·실행 (g++ / MinGW):
//     g++ -std=c++11 trapezoid_profile.cpp -o trapezoid
//     ./trapezoid            (Windows: trapezoid.exe)
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl /EHsc trapezoid_profile.cpp
//     trapezoid_profile.exe
//
// → 콘솔 요약 출력 + profile.csv(시간,거리,속도) 저장.
//   그래프는 python 폴더의 visualize.py로 그립니다(언어 무관, README 참고).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

struct Row { double t, s, v; };

std::vector<Row> profile(double L, double V, double A, double dt = 0.01) {
    double D = V * V / (2 * A);
    if (2 * D >= L) V = std::sqrt(A * L);      // triangle
    double t1 = V / A;
    double tc = std::max(L - V * V / A, 0.0) / V;
    double T = 2 * t1 + tc;
    std::vector<Row> rows; double s = 0.0;
    for (double t = 0.0; t <= T; t += dt) {
        double v = (t < t1) ? A * t
                 : (t < t1 + tc) ? V
                 : std::max(V - A * (t - t1 - tc), 0.0);
        s += v * dt;
        rows.push_back({t, s, v});
    }
    return rows;
}

int main() {
    // 영상 예제 그대로: 목표 180도, 최고 속도 초당 90도, 가속도 매초 초당 180도
    std::vector<Row> rows = profile(180.0, 90.0, 180.0);

    std::FILE* f = std::fopen("profile.csv", "w");
    if (f == nullptr) {
        std::printf("profile.csv 파일을 열 수 없습니다.\n");
        return 1;
    }
    std::fprintf(f, "t,s,v\n");
    for (const Row& r : rows)
        std::fprintf(f, "%.3f,%.4f,%.4f\n", r.t, r.s, r.v);
    std::fclose(f);
    std::printf("총 시간 = %.2f초, 최종 거리 = %.1f도  →  profile.csv 저장 완료\n",
                rows.back().t, rows.back().s);
    return 0;
}
