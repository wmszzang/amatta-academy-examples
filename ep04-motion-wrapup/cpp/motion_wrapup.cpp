// 운동계획 끝내기 - 네 지문(사다리꼴 · 3차 · 5차 · S-커브)을 한 파일로. C++ 버전.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.4
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl /EHsc motion_wrapup.cpp
//     motion_wrapup.exe
// 컴파일·실행 (g++ / MinGW):
//     g++ -std=c++11 motion_wrapup.cpp -o motion_wrapup
//     ./motion_wrapup                (Windows: motion_wrapup.exe)
//
// 출력: p1_trapezoid.csv (t,x,v,a) · p2_cubic.csv (t,q,qd,qdd) · p3_quintic.csv (t,q,qd,qdd) · p4_scurve.csv (t,x,v,a,j)
//
// 판별 규칙(영상 §갈림길): 총 시간 T가 주어지면 다항식(각가속도까지 0이면 5차, 아니면 3차),
//                         한계가 주어지면 사다리꼴 계열(저크 한계까지 있으면 S-커브).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

struct Row { double t, x, v, a, j; };   // 다항식에서는 x=각도, v=각속도, a=각가속도로 읽는다 (j 미사용)

// ── 1. 사다리꼴 (삼각형 축퇴 포함) ─────────────────────────────────────────
std::vector<Row> trapezoid(double D, double V, double A, double dt, double* T_out, double* V_out) {
    if (V * V / A > D) V = std::sqrt(A * D);          // 가속+감속 거리(V²/A)가 목표보다 길면 삼각형
    double ta = V / A;                                // 가속 시간
    double tc = (D - V * V / A) / V;                  // 등속 시간 (삼각형이면 0)
    double T = 2 * ta + tc;
    std::vector<Row> rows;
    int n = static_cast<int>(T / dt + 0.5);
    for (int i = 0; i <= n; ++i) {
        double t = i * dt, a, v, x;
        if (t < ta)           { a = A;   v = A * t;   x = A * t * t / 2; }
        else if (t < ta + tc) { a = 0.0; v = V;       x = V * ta / 2 + V * (t - ta); }
        else { double r = std::max(T - t, 0.0); a = -A; v = A * r; x = D - A * r * r / 2; }
        rows.push_back({t, x, v, a, 0.0});
    }
    *T_out = T; *V_out = V;
    return rows;
}

// ── 2. 3차 다항식 (시작·끝 속도 0) ────────────────────────────────────────
std::vector<Row> cubic(double q0, double qf, double T, double dt) {
    double d = qf - q0;                               // 이동량 = 목표 - 시작
    double a2 = 3 * d / (T * T), a3 = -2 * d / (T * T * T);
    std::vector<Row> rows;
    int n = static_cast<int>(T / dt + 0.5);
    for (int i = 0; i <= n; ++i) {
        double t = i * dt;
        rows.push_back({t, q0 + a2 * t * t + a3 * t * t * t,     // 상수항 = 시작 각도
                        2 * a2 * t + 3 * a3 * t * t, 2 * a2 + 6 * a3 * t, 0.0});
    }
    return rows;
}

// ── 3. 5차 다항식 (시작·끝 속도·가속도 0 - 10·15·6 공식) ─────────────────
std::vector<Row> quintic(double q0, double qf, double T, double dt) {
    double d = qf - q0;
    std::vector<Row> rows;
    int n = static_cast<int>(T / dt + 0.5);
    for (int i = 0; i <= n; ++i) {
        double t = i * dt, s = t / T;                 // 진행률 0 → 1
        rows.push_back({t, q0 + d * (10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5)),
                        d * (30 * s * s - 60 * std::pow(s, 3) + 30 * std::pow(s, 4)) / T,
                        d * (60 * s - 180 * s * s + 120 * std::pow(s, 3)) / (T * T), 0.0});
    }
    return rows;
}

// ── 4. S-커브 (저크 제한 7구간 - 도달 못 하는 구간은 0으로) ────────────────
std::vector<Row> scurve(double L, double V, double A, double J, double dt) {
    double tj = A / J;                                  // 저크 구간
    if (V / A < tj) { tj = std::sqrt(V / J); A = J * tj; }        // 확인 1: 가속 일정 구간이 없다
    double ta = V / A + tj, da = V * ta / 2;
    if (L < 2 * da) {                                             // 확인 2: 등속 구간이 없다
        V = (-tj + std::sqrt(tj * tj + 4 * L / A)) * A / 2;       //   V²/A + V·tj = L (근의 공식)
        if (V / A < tj) { tj = std::cbrt(L / (2 * J)); A = J * tj; V = J * tj * tj; }
        ta = V / A + tj; da = V * ta / 2;
    }
    double tc = (L - 2 * da) / V;
    double dur[7]  = {tj, ta - 2 * tj, tj, tc, tj, ta - 2 * tj, tj};
    double jerk[7] = {+J, 0.0, -J, 0.0, -J, 0.0, +J};
    std::vector<Row> rows = {{0, 0, 0, 0, +J}};
    double t = 0, x = 0, v = 0, a = 0;
    for (int k = 0; k < 7; ++k) {
        double t_end = t + dur[k], j = jerk[k];
        while (t < t_end - 1e-9) {
            double h = std::min(dt, t_end - t);         // 구간 끝은 마지막 걸음만 짧게
            x += v * h + a * h * h / 2 + j * h * h * h / 6;
            v += a * h + j * h * h / 2;
            a += j * h;
            t += h;
            rows.push_back({t, x, v, a, j});
        }
    }
    return rows;
}

static void save(const char* name, const char* header, const std::vector<Row>& rows, const char* fmt, bool with_jerk) {
    std::FILE* f = std::fopen(name, "w");
    if (f == nullptr) { std::printf("cannot open %s\n", name); return; }
    std::fprintf(f, "%s\n", header);
    for (const Row& r : rows) {
        if (with_jerk) std::fprintf(f, fmt, r.t, r.x, r.v, r.a, r.j);   // fmt = 열별 소수 자릿수(Python판과 동일)
        else           std::fprintf(f, fmt, r.t, r.x, r.v, r.a);
    }
    std::fclose(f);
    std::printf("saved: %s (%zu rows)\n", name, rows.size());
}

int main() {
    double T1, V1;
    std::vector<Row> p1 = trapezoid(150, 60, 120, 0.01, &T1, &V1);        // 지문 1
    std::printf("P1 trapezoid: total %.2f s, peak %.1f, final %.2f deg\n", T1, V1, p1.back().x);
    save("p1_trapezoid.csv", "t,x,v,a", p1, "%.2f,%.4f,%.4f,%.1f\n", false);

    std::vector<Row> p2 = cubic(30, 120, 3, 0.1);                          // 지문 2
    std::printf("P2 cubic: mid %.2f deg, final %.2f deg, accel %.1f -> %.1f\n",
                p2[p2.size() / 2].x, p2.back().x, p2.front().a, p2.back().a);
    save("p2_cubic.csv", "t,q,qd,qdd", p2, "%.1f,%.4f,%.4f,%.4f\n", false);

    std::vector<Row> p3 = quintic(-45, 45, 4, 0.2);                        // 지문 3
    std::printf("P3 quintic: mid %.2f deg, peak vel %.4f, start accel %.1f\n",
                p3[p3.size() / 2].x, p3[p3.size() / 2].v, p3.front().a);
    save("p3_quintic.csv", "t,q,qd,qdd", p3, "%.1f,%.4f,%.4f,%.4f\n", false);

    std::vector<Row> p4 = scurve(180, 90, 180, 720, 0.005);                // 지문 4
    std::printf("P4 s-curve: total %.3f s, final %.2f deg\n", p4.back().t, p4.back().x);
    save("p4_scurve.csv", "t,x,v,a,j", p4, "%.3f,%.4f,%.4f,%.4f,%.1f\n", true);
    return 0;
}
