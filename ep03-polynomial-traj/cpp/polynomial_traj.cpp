// 3차·5차 다항식 관절 궤적 — C++ 버전. cubic.csv, quintic.csv 를 만든다.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl /EHsc polynomial_traj.cpp
//     polynomial_traj.exe
// 컴파일·실행 (g++ / MinGW):
//     g++ -std=c++11 polynomial_traj.cpp -o polynomial_traj
//     ./polynomial_traj          (Windows: polynomial_traj.exe)
//
// 예제 조건(영상과 동일): 0도 → 90도, 2초. 시간 간격 0.01초.
#include <cmath>
#include <cstdio>
#include <vector>

struct Row { double t, q, qd, qdd; };   // 시각, 각도, 각속도, 각가속도

// 3차: 계수 두 줄 → 각도 / 각속도 / 각가속도
std::vector<Row> cubic(double q0, double qf, double T, double dt = 0.01) {
    double d = qf - q0, a2 = 3 * d / (T * T), a3 = -2 * d / (T * T * T);
    std::vector<Row> rows;
    for (double t = 0.0; t <= T + 1e-9; t += dt)
        rows.push_back({t, q0 + a2 * t * t + a3 * t * t * t,
                        2 * a2 * t + 3 * a3 * t * t, 2 * a2 + 6 * a3 * t});
    return rows;
}

// 5차: 정규화 시간 s = t/T 로 10·15·6 공식
std::vector<Row> quintic(double q0, double qf, double T, double dt = 0.01) {
    double d = qf - q0;
    std::vector<Row> rows;
    for (double t = 0.0; t <= T + 1e-9; t += dt) {
        double s = t / T;
        rows.push_back({t, q0 + d * (10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5)),
                        d * (30 * s * s - 60 * std::pow(s, 3) + 30 * std::pow(s, 4)) / T,
                        d * (60 * s - 180 * s * s + 120 * std::pow(s, 3)) / (T * T)});
    }
    return rows;
}

static void save(const std::vector<Row>& rows, const char* path) {
    std::FILE* f = std::fopen(path, "w");
    if (f == nullptr) return;
    std::fprintf(f, "t,q,qd,qdd\n");
    for (const Row& r : rows)
        std::fprintf(f, "%.3f,%.4f,%.4f,%.4f\n", r.t, r.q, r.qd, r.qdd);
    std::fclose(f);
}

int main() {
    const double Q0 = 0.0, QF = 90.0, T = 2.0;
    save(cubic(Q0, QF, T), "cubic.csv");
    save(quintic(Q0, QF, T), "quintic.csv");
    std::printf("saved: cubic.csv, quintic.csv\n");
    return 0;
}
