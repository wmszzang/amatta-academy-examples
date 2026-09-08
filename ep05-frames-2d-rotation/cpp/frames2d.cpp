// 프레임 · 2D 회전 · 강체 변환 — 예제 답안 (C++11)
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.5
//
// Python 판(python/frames2d.py)과 같은 계산 · 같은 출력(points.csv, angles.txt).
// 빌드: cl /EHsc frames2d.cpp   (Visual Studio 개발자 명령 프롬프트)   /   g++ -std=c++11 frames2d.cpp -o frames2d
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

const double PI = 3.14159265358979323846;   // M_PI 는 표준이 아니라 직접 정의

struct Pt { double x, y; };
struct Rot { double c, s; };                   // R(θ) = [[c, -s], [s, c]]  — 첫째 열 (c, s) = 돌아간 x축, 둘째 열 (−s, c) = 돌아간 y축

Rot rot2d(double deg) { double t = deg * PI / 180.0; return { std::cos(t), std::sin(t) }; }   // sin/cos 는 라디안만
Pt apply(Rot R, Pt p) { return { R.c * p.x - R.s * p.y, R.s * p.x + R.c * p.y }; }

Pt to_base(Rot R, Pt t, Pt p) {                // 툴 → 베이스 : p' = R p + t (회전 후 이동)
    Pt q = apply(R, p); return { q.x + t.x, q.y + t.y };
}
Pt to_tool(Rot R, Pt t, Pt p) {                // 베이스 → 툴 : 이동량 빼고 Rᵀ 곱하기 (직교성 — 역행렬 = 전치)
    double dx = p.x - t.x, dy = p.y - t.y;
    return { R.c * dx + R.s * dy, -R.s * dx + R.c * dy };
}
std::vector<Pt> rigid_2d(const std::vector<Pt>& pts, double deg, double tx, double ty) {
    Rot R = rot2d(deg); std::vector<Pt> out;
    for (const Pt& p : pts) out.push_back(to_base(R, { tx, ty }, p));
    return out;
}
double orth_check(Rot R, double rtr[2][2]) {   // RᵀR — 항등행렬과의 최대 오차 반환
    double a = R.c, b = -R.s, c = R.s, d = R.c;
    rtr[0][0] = a * a + c * c; rtr[0][1] = a * b + c * d;
    rtr[1][0] = b * a + d * c; rtr[1][1] = b * b + d * d;
    double e = std::fabs(rtr[0][0] - 1);
    e = std::fmax(e, std::fabs(rtr[0][1])); e = std::fmax(e, std::fabs(rtr[1][0])); e = std::fmax(e, std::fabs(rtr[1][1] - 1));
    return e;
}
double wrap(double a) {                        // [−π, π] 로 접기 — fmod 는 부호를 유지하므로 음수면 한 바퀴 더한다
    double m = std::fmod(a + PI, 2 * PI); if (m < 0) m += 2 * PI; return m - PI;
}
double wrap_deg(double d) { double m = std::fmod(d + 180.0, 360.0); if (m < 0) m += 360.0; return m - 180.0; }
double ang_err(double target, double current) { return wrap(target - current); }
double circ_mean_deg(const std::vector<double>& degs) {   // atan2(Σ sin, Σ cos) — y 먼저, x 나중
    double s = 0, c = 0;
    for (double d : degs) { s += std::sin(d * PI / 180.0); c += std::cos(d * PI / 180.0); }
    return std::atan2(s, c) * 180.0 / PI;
}

int main() {
    std::vector<Pt> square = { {0, 0}, {1, 0}, {1, 1}, {0, 1} };
    double deg = 30.0; Pt t = { 1.0, 0.5 };
    Rot R = rot2d(deg);
    std::vector<Pt> moved = rigid_2d(square, deg, t.x, t.y);

    std::printf("R(30) = [[%.4f, %.4f], [%.4f, %.4f]]\n", R.c, -R.s, R.s, R.c);
    std::printf("square 30deg t=(1,0.5):");
    for (const Pt& p : moved) std::printf(" (%.4f, %.4f)", p.x, p.y);
    std::printf("\n");
    {
        std::ofstream f("points.csv");
        f << "x,y,xr,yr\n";
        char buf[96];
        for (size_t i = 0; i < square.size(); ++i) {
            std::snprintf(buf, sizeof buf, "%.4f,%.4f,%.4f,%.4f\n", square[i].x, square[i].y, moved[i].x, moved[i].y);
            f << buf;
        }
    }
    Pt pb = to_base(R, t, { 1, 0 }), pt = to_tool(R, t, { 2, 1 }), back = to_base(R, t, pt);
    std::printf("tool(1,0)->base(%.4f, %.4f)\n", pb.x, pb.y);
    std::printf("base(2,1)->tool(%.4f, %.4f)\n", pt.x, pt.y);
    std::printf("->back(%.4f, %.4f)\n", back.x, back.y);
    Pt mr = apply(R, { 1 + t.x, 0 + t.y });
    std::printf("move-then-rotate (1,0): (%.4f, %.4f)   (rotate-then-move: (%.4f, %.4f))\n", mr.x, mr.y, moved[1].x, moved[1].y);

    double rtr[2][2]; double err = orth_check(R, rtr);
    std::printf("RtR = [[%.4f, %.4f], [%.4f, %.4f]]  max_err=%.1e\n", rtr[0][0], rtr[0][1], rtr[1][0], rtr[1][1], err);
    std::printf("det = %.4f   trace = %.4f\n", R.c * R.c + R.s * R.s, 2 * R.c);

    char buf[64]; std::vector<std::string> lines;
    std::snprintf(buf, sizeof buf, "ang_err(3.0,-3.0)=%.4f", ang_err(3.0, -3.0)); lines.push_back(buf);
    std::snprintf(buf, sizeof buf, "ang_err(0.1,-0.1)=%.4f", ang_err(0.1, -0.1)); lines.push_back(buf);
    std::snprintf(buf, sizeof buf, "circ_mean([170,-170])=%.4f", circ_mean_deg({ 170, -170 })); lines.push_back(buf);
    std::snprintf(buf, sizeof buf, "circ_mean([10,-10,350])=%.4f", circ_mean_deg({ 10, -10, 350 })); lines.push_back(buf);
    std::snprintf(buf, sizeof buf, "wrap(359deg)=%.4fdeg", wrap_deg(359.0)); lines.push_back(buf);
    lines.push_back(std::string("orth_check(30)=") + (err < 1e-12 ? "identity(max_err<1e-12)" : "FAIL"));
    std::ofstream g("angles.txt");
    for (const std::string& ln : lines) { std::printf("%s\n", ln.c_str()); g << ln << "\n"; }
    std::printf("(naive) 3.0-(-3.0)=%.4f   mean([170,-170])=%.4f   mean([10,-10,350])=%.4f\n", 6.0, 0.0, (10 - 10 + 350) / 3.0);
    std::printf("saved: points.csv, angles.txt\n");
    return 0;
}
