// EP.14 역기구학·자코비안 끝내기 — C++11, 파이썬과 같은 로직·같은 출력.
// 빌드: g++ -std=c++11 kin14.cpp -o kin14.exe
// 속도(v = J qdot)와 토크(tau = J^T F)를 나란히 두어 전치 여부만 다르다는 것을 보인다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

static const double TOL = 1e-6;
static const double G = 9.81;
static const double PI = 3.14159265358979323846;

struct Vec2 { double x, y; };
struct Mat2 { double m[2][2]; };

static double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

static Vec2 fk2(double a1, double a2, double t1, double t2) {
    Vec2 p;
    p.x = a1 * std::cos(t1) + a2 * std::cos(t1 + t2);
    p.y = a1 * std::sin(t1) + a2 * std::sin(t1 + t2);
    return p;
}

static Mat2 jac(double a1, double a2, double t1, double t2) {
    Mat2 J;
    double s1 = std::sin(t1), c1 = std::cos(t1);
    double s12 = std::sin(t1 + t2), c12 = std::cos(t1 + t2);
    J.m[0][0] = -a1 * s1 - a2 * s12;  J.m[0][1] = -a2 * s12;
    J.m[1][0] =  a1 * c1 + a2 * c12;  J.m[1][1] =  a2 * c12;
    return J;
}

static double det2(const Mat2& J) {
    return J.m[0][0] * J.m[1][1] - J.m[0][1] * J.m[1][0];
}

// v = J qdot  — 행에 관절속도를 곱해 더한다.
static Vec2 velocity(const Mat2& J, Vec2 dq) {
    Vec2 v;
    v.x = J.m[0][0] * dq.x + J.m[0][1] * dq.y;
    v.y = J.m[1][0] * dq.x + J.m[1][1] * dq.y;
    return v;
}

// tau = J^T F — 행과 열을 바꿔 곱한다.
static Vec2 torque(const Mat2& J, Vec2 F) {
    Vec2 t;
    t.x = J.m[0][0] * F.x + J.m[1][0] * F.y;
    t.y = J.m[0][1] * F.x + J.m[1][1] * F.y;
    return t;
}

static void svd2(const Mat2& J, double* smax, double* smin) {
    double a = J.m[0][0] * J.m[0][0] + J.m[1][0] * J.m[1][0];
    double b = J.m[0][0] * J.m[0][1] + J.m[1][0] * J.m[1][1];
    double d = J.m[0][1] * J.m[0][1] + J.m[1][1] * J.m[1][1];
    double tr = a + d, dt = a * d - b * b;
    double disc = std::sqrt(std::max(0.0, tr * tr / 4 - dt));
    *smax = std::sqrt(std::max(0.0, tr / 2 + disc));
    *smin = std::sqrt(std::max(0.0, tr / 2 - disc));
}

static int ik2(double a1, double a2, double x, double y, int elbowUp, Vec2* out) {
    double r2 = x * x + y * y, r = std::sqrt(r2);
    if (r < std::fabs(a1 - a2) - 1e-12 || r > a1 + a2 + 1e-12) return 0;
    double c2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;            // 부동소수 미세 오차만 클램핑
    double s2 = std::sqrt(std::max(0.0, 1 - c2 * c2));
    if (elbowUp) s2 = -s2;
    out->y = std::atan2(s2, c2);
    out->x = std::atan2(y, x) - std::atan2(a2 * s2, a1 + a2 * c2);
    out->x = wrapAngle(out->x);
    out->y = wrapAngle(out->y);
    return 1;
}

struct NumResult { int converged, checks, updates; double t1raw, t2raw, t1w, t2w; };

static NumResult newtonDls(double gx, double gy, double t1, double t2,
                           double a1, double a2, double lam) {
    NumResult R = {0, 0, 0, 0, 0, 0, 0};
    for (int it = 1; it <= 200; ++it) {
        Vec2 p = fk2(a1, a2, t1, t2);
        double ex = gx - p.x, ey = gy - p.y;
        double e = std::sqrt(ex * ex + ey * ey);
        if (e < TOL) {
            R.converged = 1; R.checks = it; R.updates = it - 1;
            R.t1raw = t1; R.t2raw = t2;
            R.t1w = wrapAngle(t1); R.t2w = wrapAngle(t2);
            return R;
        }
        Mat2 J = jac(a1, a2, t1, t2);
        if (lam > 0) {
            double A = J.m[0][0] * J.m[0][0] + J.m[1][0] * J.m[1][0] + lam * lam;
            double B = J.m[0][0] * J.m[0][1] + J.m[1][0] * J.m[1][1];
            double C = J.m[0][1] * J.m[0][1] + J.m[1][1] * J.m[1][1] + lam * lam;
            double g0 = J.m[0][0] * ex + J.m[1][0] * ey;
            double g1 = J.m[0][1] * ex + J.m[1][1] * ey;
            double D = A * C - B * B;
            if (D == 0) { R.checks = it; R.updates = it - 1; return R; }
            t1 += (C * g0 - B * g1) / D;
            t2 += (-B * g0 + A * g1) / D;
        } else {
            double d = det2(J);
            if (d == 0) { R.checks = it; R.updates = it - 1; return R; }
            t1 += (J.m[1][1] * ex - J.m[0][1] * ey) / d;
            t2 += (-J.m[1][0] * ex + J.m[0][0] * ey) / d;
        }
    }
    R.checks = 200; R.updates = 199;
    return R;
}

static double crossOAB(Vec2 o, Vec2 a, Vec2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

static bool lessPt(const Vec2& a, const Vec2& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

static std::vector<Vec2> hull(std::vector<Vec2> P) {
    std::sort(P.begin(), P.end(), lessPt);
    P.erase(std::unique(P.begin(), P.end(),
                        [](const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; }),
            P.end());
    if (P.size() < 3) return P;
    std::vector<Vec2> ch;
    for (int pass = 0; pass < 2; ++pass) {
        size_t start = ch.size() + 2;
        if (pass == 0) {
            for (size_t i = 0; i < P.size(); ++i) {
                while (ch.size() >= start && crossOAB(ch[ch.size() - 2], ch.back(), P[i]) <= 0) ch.pop_back();
                ch.push_back(P[i]);
            }
        } else {
            for (size_t i = P.size(); i-- > 0;) {
                while (ch.size() >= start && crossOAB(ch[ch.size() - 2], ch.back(), P[i]) <= 0) ch.pop_back();
                ch.push_back(P[i]);
            }
        }
        ch.pop_back();
    }
    return ch;
}

static double shoelace(const std::vector<Vec2>& P) {
    double s = 0;
    for (size_t i = 0; i < P.size(); ++i) {
        const Vec2& a = P[i];
        const Vec2& b = P[(i + 1) % P.size()];
        s += a.x * b.y - b.x * a.y;
    }
    return std::fabs(s) / 2;
}

static bool inPoly(Vec2 p, const std::vector<Vec2>& poly) {
    bool inside = false;
    for (size_t i = 0; i < poly.size(); ++i) {
        Vec2 a = poly[i], b = poly[(i + 1) % poly.size()];
        if ((a.y > p.y) != (b.y > p.y)) {
            double xc = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (p.x < xc) inside = !inside;
        }
    }
    return inside;
}

int main() {
    const double a1 = 2.0, a2 = 1.5;

    printf("=== (1) 해석 IK ===\n");
    Vec2 s;
    ik2(1.0, 1.0, 1.5, 1.0, 0, &s);
    Vec2 f = fk2(1.0, 1.0, s.x, s.y);
    printf("#222 th=(%.4f, %.4f) deg  FK=(%.4f, %.4f)\n",
           s.x * 180 / PI, s.y * 180 / PI, f.x, f.y);
    for (int up = 0; up < 2; ++up) {
        ik2(a1, a2, 2.5, 1.5, up, &s);
        f = fk2(a1, a2, s.x, s.y);
        printf("#242 %-4s th=(%.4f, %.4f) rad  FK=(%.4f, %.4f)\n",
               up ? "up" : "down", s.x, s.y, f.x, f.y);
    }
    double tx[2] = {3.0, 0.3}, ty[2] = {2.0, 0.2};
    for (int i = 0; i < 2; ++i) {
        double r = std::sqrt(tx[i] * tx[i] + ty[i] * ty[i]);
        double c2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2);
        printf("도달 판정 (%.1f, %.1f) r=%.4f cos(th2)=%.4f -> %s\n",
               tx[i], ty[i], r, c2,
               (r >= std::fabs(a1 - a2) && r <= a1 + a2) ? "REACHABLE" : "UNREACHABLE");
    }
    printf("#477 6.0 -> wrap %.4f,  7.3208 -> wrap %.4f\n",
           wrapAngle(6.0), wrapAngle(7.3208));

    printf("\n=== (2) 작업공간 ===\n");
    ik2(a1, a2, 2.5, 1.5, 0, &s);
    Mat2 J257 = jac(a1, a2, s.x, s.y);
    double smax, smin;
    svd2(J257, &smax, &smin);
    printf("#257 r=%.4f REACHABLE  w=%.4f  smax=%.4f  smin=%.4f  kappa=%.4f\n",
           std::sqrt(2.5 * 2.5 + 1.5 * 1.5), std::fabs(det2(J257)), smax, smin, smax / smin);
    printf("     환형 넓이 = %.4f m^2\n",
           PI * ((a1 + a2) * (a1 + a2) - (a1 - a2) * (a1 - a2)));
    std::vector<Vec2> pts;
    for (int i = 0; i < 25; ++i)
        for (int j = 0; j < 25; ++j) {
            Vec2 p = fk2(a1, a2, (90.0 * i / 24) * PI / 180, (120.0 * j / 24) * PI / 180);
            pts.push_back(p);
        }
    std::vector<Vec2> H = hull(pts);
    std::vector<Vec2> fence;
    double fx[5] = {-1.8, 3.8, 3.8, 1.0, -1.8}, fy[5] = {-0.4, -0.4, 2.4, 3.8, 2.4};
    for (int i = 0; i < 5; ++i) { Vec2 v = {fx[i], fy[i]}; fence.push_back(v); }
    int outside = 0;
    for (size_t i = 0; i < pts.size(); ++i) if (!inPoly(pts[i], fence)) ++outside;
    printf("#497 스윕 %d점 -> 껍질 %d정점\n", (int)pts.size(), (int)H.size());
    printf("#287 껍질 넓이 = %.4f, 펜스 넓이 = %.4f, 펜스 밖 = %d/%d\n",
           shoelace(H), shoelace(fence), outside, (int)pts.size());

    printf("\n=== (3) 속도·힘 ===\n");
    Mat2 J = jac(a1, a2, 0.5, 0.5);
    printf("J = [[%.4f, %.4f], [%.4f, %.4f]]  det J = %.4f\n",
           J.m[0][0], J.m[0][1], J.m[1][0], J.m[1][1], det2(J));
    Vec2 dq = {0.4, -0.3};
    Vec2 v = velocity(J, dq);
    printf("#239 v = (%.4f, %.4f) m/s  |v| = %.4f\n", v.x, v.y, std::sqrt(v.x * v.x + v.y * v.y));
    Vec2 F = {3.0, -1.0};
    Vec2 t = torque(J, F);
    Vec2 wrongT = velocity(J, F);
    printf("#260 tau = J^T F = (%.4f, %.4f) N.m\n", t.x, t.y);
    printf("     전치 누락 J F = (%.4f, %.4f), 차이 크기 = %.4f\n",
           wrongT.x, wrongT.y,
           std::sqrt((t.x - wrongT.x) * (t.x - wrongT.x) + (t.y - wrongT.y) * (t.y - wrongT.y)));
    printf("     가상일 F.v = %.6f W, tau.qdot = %.6f W\n",
           F.x * v.x + F.y * v.y, t.x * dq.x + t.y * dq.y);
    double t2s[5] = {0.5, 0.2, 0.05, 0.01, 0.001};
    for (int i = 0; i < 5; ++i) {
        Mat2 Ji = jac(a1, a2, 0.5, t2s[i]);
        double d = det2(Ji);
        double q1 = (Ji.m[1][1] * 0.0 - Ji.m[0][1] * 0.2) / d;
        double q2 = (-Ji.m[1][0] * 0.0 + Ji.m[0][0] * 0.2) / d;
        printf("     th2=%.3f  det J=%.6f  |qdot|=%.4f rad/s\n",
               t2s[i], d, std::sqrt(q1 * q1 + q2 * q2));
    }
    double tg2 = (2.0 * a2 / 2) * G * std::cos(1.0);
    double tg1 = (3.0 * a1 / 2 + 2.0 * a1) * G * std::cos(0.5) + tg2;
    printf("중력 보상 tau_g = (%.4f, %.4f) N.m, 지령 = (%.4f, %.4f) N.m\n",
           tg1, tg2, tg1 + t.x, tg2 + t.y);

    printf("\n=== (4) 수치 IK ===\n");
    double st1[4] = {0.5, 0.0, 0.0, 0.1}, st2[4] = {0.5, 0.0, 0.0, 0.05};
    double lams[4] = {0.0, 0.0, 0.05, 0.0};
    for (int i = 0; i < 4; ++i) {
        NumResult R = newtonDls(2.5, 1.5, st1[i], st2[i], a1, a2, lams[i]);
        if (R.converged)
            printf("lam=%.2f 시작 (%.2f, %.2f)  검사 %2d회 / 갱신 %2d회  raw=(%.4f, %.4f) wrap=(%.4f, %.4f)\n",
                   lams[i], st1[i], st2[i], R.checks, R.updates, R.t1raw, R.t2raw, R.t1w, R.t2w);
        else
            printf("lam=%.2f 시작 (%.2f, %.2f)  검사 %2d회 / 갱신 %2d회  실패: det J = 0\n",
                   lams[i], st1[i], st2[i], R.checks, R.updates);
    }

    printf("\n=== (5) 안정성 ===\n");
    const char* names[2] = {"folded", "extended"};
    double p1[2] = {0.5, 0.1}, p2[2] = {0.5, 0.05};
    for (int i = 0; i < 2; ++i) {
        double ex = a1 * std::cos(p1[i]), ey = a1 * std::sin(p1[i]);
        Vec2 tip = fk2(a1, a2, p1[i], p2[i]);
        double total = 30.0 + 3.0 + 2.0 + 1.0;
        double cx = (3.0 * ex / 2 + 2.0 * (ex + tip.x) / 2 + 1.0 * tip.x) / total;
        double cy = (3.0 * ey / 2 + 2.0 * (ey + tip.y) / 2 + 1.0 * tip.y) / total;
        double margin = std::min(0.30 - std::fabs(cx), 0.30 - std::fabs(cy));
        printf("#484 %-9s CoM 투영 (%.4f, %.4f)  여유 %+.4f m  %s\n",
               names[i], cx, cy, margin, margin >= 0 ? "안정" : "전도");
    }
    return 0;
}
