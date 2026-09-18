// EP.13 정역학 (C++) — tau = J^T F · 중력 보상 · 지지 다각형 안정 여유
// 빌드: g++ -std=c++11 cpp/statics.cpp -o statics_cpp.exe
// 실행: ./statics_cpp.exe
#include <cstdio>
#include <cmath>
#include <vector>
#include <utility>

namespace {
const double G = 9.81;
const double PI = 3.14159265358979323846;

void jac(double l1, double l2, double t1, double t2, double J[2][2]) {
    double s1 = std::sin(t1), c1 = std::cos(t1);
    double s12 = std::sin(t1 + t2), c12 = std::cos(t1 + t2);
    J[0][0] = -l1 * s1 - l2 * s12; J[0][1] = -l2 * s12;
    J[1][0] =  l1 * c1 + l2 * c12; J[1][1] =  l2 * c12;
}

// 전치: 열이 아니라 행을 읽는다.
void torque(const double J[2][2], double fx, double fy, double t[2]) {
    t[0] = J[0][0] * fx + J[1][0] * fy;
    t[1] = J[0][1] * fx + J[1][1] * fy;
}

// 전치를 빠뜨린 오답.
void wrongTorque(const double J[2][2], double fx, double fy, double t[2]) {
    t[0] = J[0][0] * fx + J[0][1] * fy;
    t[1] = J[1][0] * fx + J[1][1] * fy;
}

double grav1(double a1, double lc1, double lc2, double m1, double m2,
             double t1, double t2, double g) {
    return (m1 * lc1 + m2 * a1) * g * std::cos(t1) + m2 * lc2 * g * std::cos(t1 + t2);
}

double grav2(double lc2, double m2, double t1, double t2, double g) {
    return m2 * lc2 * g * std::cos(t1 + t2);
}

std::pair<bool, double> supportMargin(const std::vector<std::pair<double, double> >& poly,
                                      double px, double py) {
    double best = 0.0;
    bool first = true;
    size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        double ax = poly[i].first, ay = poly[i].second;
        double bx = poly[(i + 1) % n].first, by = poly[(i + 1) % n].second;
        double ex = bx - ax, ey = by - ay;
        double cross = ex * (py - ay) - ey * (px - ax);
        double d = cross / std::sqrt(ex * ex + ey * ey);
        if (first || d < best) { best = d; first = false; }
    }
    return std::make_pair(best >= 0.0, best);
}
}  // namespace

int main() {
    const double a1 = 2.0, a2 = 1.5, t1 = 0.5, t2 = 0.5;
    double J[2][2], tau[2], bad[2];
    jac(a1, a2, t1, t2, J);

    std::printf("== #260 Jacobian transpose (a 2,1.5 / theta 0.5,0.5 / F 3,-1 N) ==\n");
    std::printf("  J row0  %.4f %.4f\n", J[0][0], J[0][1]);
    std::printf("  J row1  %.4f %.4f\n", J[1][0], J[1][1]);
    std::printf("  det J   %.4f\n", J[0][0] * J[1][1] - J[0][1] * J[1][0]);
    torque(J, 3.0, -1.0, tau);
    wrongTorque(J, 3.0, -1.0, bad);
    std::printf("  tau = J^T F  (%.4f, %.4f) N*m\n", tau[0], tau[1]);
    std::printf("  no transpose (%.4f, %.4f)  <- wrong\n", bad[0], bad[1]);

    std::printf("== virtual work check (qdot 0.3,-0.7) ==\n");
    double qd1 = 0.3, qd2 = -0.7;
    double vx = J[0][0] * qd1 + J[0][1] * qd2, vy = J[1][0] * qd1 + J[1][1] * qd2;
    std::printf("  xdot (%.4f, %.4f)  F.xdot %.10f  tau.qdot %.10f\n",
                vx, vy, 3.0 * vx + (-1.0) * vy, tau[0] * qd1 + tau[1] * qd2);

    std::printf("== gravity compensation (m 3,2 kg / lc mid-link / g 9.81) ==\n");
    const double lc1 = a1 / 2, lc2 = a2 / 2, m1 = 3.0, m2 = 2.0;
    const double poses[3][2] = {{0.5, 0.5}, {0.0, 0.0}, {PI / 2, 0.0}};
    const char* names[3] = {"base 0.5,0.5", "flat 0,0    ", "up pi/2,0   "};
    for (int k = 0; k < 3; ++k)
        std::printf("  %s  (%.4f, %.4f) N*m\n", names[k],
                    grav1(a1, lc1, lc2, m1, m2, poses[k][0], poses[k][1], G),
                    grav2(lc2, m2, poses[k][0], poses[k][1], G));
    double pay[2];
    torque(J, 0.0, 2.0 * G, pay);
    double g1 = grav1(a1, lc1, lc2, m1, m2, t1, t2, G), g2 = grav2(lc2, m2, t1, t2, G);
    std::printf("  2 kg payload  (%.4f, %.4f) N*m\n", pay[0], pay[1]);
    std::printf("  total         (%.4f, %.4f) N*m\n", g1 + pay[0], g2 + pay[1]);

    std::printf("== #484 support polygon static stability margin ==\n");
    std::vector<std::pair<double, double> > square, tri;
    square.push_back(std::make_pair(0.0, 0.0)); square.push_back(std::make_pair(1.0, 0.0));
    square.push_back(std::make_pair(1.0, 1.0)); square.push_back(std::make_pair(0.0, 1.0));
    tri.push_back(std::make_pair(0.0, 0.0)); tri.push_back(std::make_pair(2.0, 0.0));
    tri.push_back(std::make_pair(1.0, 1.5));
    const double pts[3][2] = {{0.5, 0.5}, {0.9, 0.5}, {1.2, 0.5}};
    for (int k = 0; k < 3; ++k) {
        std::pair<bool, double> r = supportMargin(square, pts[k][0], pts[k][1]);
        std::printf("  square P(%.1f, %.1f) -> %s, margin %.4f\n",
                    pts[k][0], pts[k][1], r.first ? "stable  " : "unstable", r.second);
    }
    std::pair<bool, double> rt = supportMargin(tri, 1.0, 0.4);
    std::printf("  triangle P(1.0, 0.4) -> %s, margin %.4f\n",
                rt.first ? "stable  " : "unstable", rt.second);
    return 0;
}
