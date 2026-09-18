// EP.13 수치 역기구학 (C++) — 뉴턴-랩슨 · DLS · 의사역행렬 · 널공간
// 빌드: g++ -std=c++11 cpp/numeric_ik.cpp -o numeric_ik_cpp.exe
// 실행: ./numeric_ik_cpp.exe
#include <cstdio>
#include <cmath>
#include <array>
#include <vector>

namespace {
const double TOL = 1e-6;
const int ITMAX = 200;
const double EPS_SING = 1e-12;
const double PI = 3.14159265358979323846;

typedef std::array<std::array<double, 2>, 2> Mat22;
typedef std::array<std::array<double, 3>, 2> Mat23;
typedef std::array<double, 3> Vec3;

std::array<double, 2> fk2(double l1, double l2, double t1, double t2) {
    return {{l1 * std::cos(t1) + l2 * std::cos(t1 + t2),
             l1 * std::sin(t1) + l2 * std::sin(t1 + t2)}};
}

Mat22 jacobian(double l1, double l2, double t1, double t2) {
    double s1 = std::sin(t1), c1 = std::cos(t1);
    double s12 = std::sin(t1 + t2), c12 = std::cos(t1 + t2);
    Mat22 J;
    J[0][0] = -l1 * s1 - l2 * s12; J[0][1] = -l2 * s12;
    J[1][0] =  l1 * c1 + l2 * c12; J[1][1] =  l2 * c12;
    return J;
}

double wrap(double a) {
    double w = std::fmod(a + PI, 2.0 * PI);
    if (w < 0) w += 2.0 * PI;
    return w - PI;
}

struct Result { double t1, t2; int updates; bool singular; double maxErr; std::vector<double> errs; };

// lam < 0 이면 무감쇠 뉴턴, lam >= 0 이면 DLS.
Result solve(double gx, double gy, double t1, double t2,
             double l1, double l2, double step, double lam) {
    Result r{t1, t2, 0, false, 0.0, {}};
    int it = 0;
    for (; it < ITMAX; ++it) {
        std::array<double, 2> p = fk2(l1, l2, t1, t2);
        double ex = gx - p[0], ey = gy - p[1];
        double err = std::sqrt(ex * ex + ey * ey);
        r.errs.push_back(err);
        if (err > r.maxErr) r.maxErr = err;
        if (err < TOL) break;
        Mat22 J = jacobian(l1, l2, t1, t2);
        if (lam < 0.0) {
            double d = J[0][0] * J[1][1] - J[0][1] * J[1][0];
            if (std::fabs(d) < EPS_SING) { r.singular = true; break; }
            t1 += step * ( J[1][1] * ex - J[0][1] * ey) / d;
            t2 += step * (-J[1][0] * ex + J[0][0] * ey) / d;
        } else {
            double a11 = J[0][0] * J[0][0] + J[0][1] * J[0][1] + lam * lam;
            double a12 = J[0][0] * J[1][0] + J[0][1] * J[1][1];
            double a22 = J[1][0] * J[1][0] + J[1][1] * J[1][1] + lam * lam;
            double det = a11 * a22 - a12 * a12;
            double u = ( a22 * ex - a12 * ey) / det;
            double v = (-a12 * ex + a11 * ey) / det;
            t1 += J[0][0] * u + J[1][0] * v;
            t2 += J[0][1] * u + J[1][1] * v;
        }
    }
    r.t1 = t1; r.t2 = t2; r.updates = it;
    return r;
}

Mat23 jacobian3(const Vec3& a, const Vec3& th) {
    Vec3 cum{{th[0], th[0] + th[1], th[0] + th[1] + th[2]}};
    Mat23 J{};
    for (int k = 0; k < 3; ++k)
        for (int i = k; i < 3; ++i) {
            J[0][k] -= a[i] * std::sin(cum[i]);
            J[1][k] += a[i] * std::cos(cum[i]);
        }
    return J;
}

Vec3 pinvVel(const Mat23& J, double xdx, double xdy) {
    double b11 = 0, b12 = 0, b22 = 0;
    for (int k = 0; k < 3; ++k) {
        b11 += J[0][k] * J[0][k]; b12 += J[0][k] * J[1][k]; b22 += J[1][k] * J[1][k];
    }
    double det = b11 * b22 - b12 * b12;
    double u = ( b22 * xdx - b12 * xdy) / det;
    double v = (-b12 * xdx + b11 * xdy) / det;
    Vec3 q{};
    for (int k = 0; k < 3; ++k) q[k] = J[0][k] * u + J[1][k] * v;
    return q;
}

Vec3 nullDir(const Mat23& J) {
    Vec3 n{{J[0][1] * J[1][2] - J[0][2] * J[1][1],
            J[0][2] * J[1][0] - J[0][0] * J[1][2],
            J[0][0] * J[1][1] - J[0][1] * J[1][0]}};
    double s = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    for (int k = 0; k < 3; ++k) n[k] /= s;
    return n;
}
}  // namespace

int main() {
    std::printf("== #253 Newton-Raphson (goal 2.5,1.5 / init 0.5,0.5 / step 1.0) ==\n");
    Result r = solve(2.5, 1.5, 0.5, 0.5, 2.0, 1.5, 1.0, -1.0);
    for (size_t i = 0; i < r.errs.size(); ++i) std::printf("    it %zu  err %.6g\n", i, r.errs[i]);
    std::array<double, 2> p = fk2(2.0, 1.5, r.t1, r.t2);
    std::printf("  theta (%.4f, %.4f)  updates %d  FK (%.4f, %.4f)\n", r.t1, r.t2, r.updates, p[0], p[1]);

    std::printf("== step sweep ==\n");
    const double steps[4] = {1.0, 0.7, 0.5, 0.2};
    for (int k = 0; k < 4; ++k) {
        Result s = solve(2.5, 1.5, 0.5, 0.5, 2.0, 1.5, steps[k], -1.0);
        std::printf("  step %.1f -> %d updates\n", steps[k], s.updates);
    }

    std::printf("== singular start, undamped (goal 3.4,0.15 / init 0,0) ==\n");
    Result d0 = solve(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, 1.0, -1.0);
    std::printf("  status %s\n", d0.singular ? "singular (stopped at first division)" : "converged");

    std::printf("== nudged start 0.001 — blow-up ==\n");
    Result b = solve(3.4, 0.15, 0.0, 0.001, 2.0, 1.5, 1.0, -1.0);
    p = fk2(2.0, 1.5, b.t1, b.t2);
    std::printf("  theta (%.4f, %.4f) rad = (%.1f, %.1f) deg  max err %.4f\n",
                b.t1, b.t2, b.t1 * 180.0 / PI, b.t2 * 180.0 / PI, b.maxErr);
    std::printf("  FK (%.4f, %.4f)  wrapped (%.4f, %.4f)\n", p[0], p[1], wrap(b.t1), wrap(b.t2));

    std::printf("== DLS lam 0.1, same singular start ==\n");
    Result dl = solve(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, 1.0, 0.1);
    for (size_t i = 0; i < dl.errs.size(); ++i) std::printf("    it %zu  err %.6g\n", i, dl.errs[i]);
    p = fk2(2.0, 1.5, dl.t1, dl.t2);
    std::printf("  theta (%.4f, %.4f)  updates %d  FK (%.4f, %.4f)\n", dl.t1, dl.t2, dl.updates, p[0], p[1]);

    std::printf("== lambda sweep ==\n");
    const double lams[5] = {0.01, 0.05, 0.1, 0.3, 1.0};
    for (int k = 0; k < 5; ++k) {
        Result s = solve(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, 1.0, lams[k]);
        std::printf("  lam %.2f -> %3d updates   max err %.4f\n", lams[k], s.updates, s.maxErr);
    }

    std::printf("== 3-link pseudoinverse / null space ==\n");
    Vec3 a3{{2.0, 1.5, 1.0}}, th3{{0.5, 0.5, 0.5}};
    Mat23 J3 = jacobian3(a3, th3);
    std::printf("  J row0 %.4f %.4f %.4f\n", J3[0][0], J3[0][1], J3[0][2]);
    std::printf("  J row1 %.4f %.4f %.4f\n", J3[1][0], J3[1][1], J3[1][2]);
    Vec3 q = pinvVel(J3, 0.0, 1.0), n = nullDir(J3);
    std::printf("  qdot %.4f %.4f %.4f  norm %.4f\n", q[0], q[1], q[2],
                std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]));
    std::printf("  n    %.4f %.4f %.4f\n", n[0], n[1], n[2]);
    for (int k = 0; k < 3; ++k) {
        double alpha = 0.5 * k, vx = 0, vy = 0, nr = 0;
        for (int i = 0; i < 3; ++i) {
            double qq = q[i] + alpha * n[i];
            vx += J3[0][i] * qq; vy += J3[1][i] * qq; nr += qq * qq;
        }
        std::printf("  alpha %.1f  norm %.4f  tip (%.4f, %.4f)\n", alpha, std::sqrt(nr), vx, vy);
    }
    return 0;
}
