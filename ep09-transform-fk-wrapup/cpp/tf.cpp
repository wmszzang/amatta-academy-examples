// EP.9 지문 A~E + 경계 2문제 — C++ 최소 구현 (표준 라이브러리만).
// g++ -std=c++11 cpp/tf.cpp -o tf-cpp.exe; .\tf-cpp.exe   → python/run_all.py 와 같은 출력
#include <cmath>
#include <cstdio>
#include <vector>

namespace tf {
const double PI = 3.14159265358979323846;
double rad(double d) { return d * PI / 180.0; }
double deg(double r) { return r * 180.0 / PI; }

struct P2 { double x, y; };
struct P3 { double x, y, z; };
struct T4 { double m[4][4]; };

// ---------- 각도 도구 ----------
double wrap(double d) { double r = std::fmod(d + 180.0, 360.0); if (r < 0) r += 360.0; return r - 180.0; }
double circular_mean(const std::vector<double>& h) {
    double s = 0, c = 0;
    for (double a : h) { s += std::sin(rad(a)); c += std::cos(rad(a)); }
    return deg(std::atan2(s, c));
}

// ---------- 2D 강체변환 ----------
std::vector<P2> rigid2d(const std::vector<P2>& pts, double a, P2 t) {
    double c = std::cos(rad(a)), s = std::sin(rad(a));
    std::vector<P2> out;
    for (const P2& p : pts) out.push_back({c * p.x - s * p.y + t.x, s * p.x + c * p.y + t.y});
    return out;
}
void inv2d(double a, P2 t, double& ia, P2& it) {
    double c = std::cos(rad(a)), s = std::sin(rad(a));
    ia = -a; it = {-(c * t.x + s * t.y), -(-s * t.x + c * t.y)};
}

// ---------- 4x4 동차변환 ----------
T4 homog_z(double a, P3 t) { double c = std::cos(rad(a)), s = std::sin(rad(a)); return {{{c, -s, 0, t.x}, {s, c, 0, t.y}, {0, 0, 1, t.z}, {0, 0, 0, 1}}}; }
T4 homog_x(double a, P3 t) { double c = std::cos(rad(a)), s = std::sin(rad(a)); return {{{1, 0, 0, t.x}, {0, c, -s, t.y}, {0, s, c, t.z}, {0, 0, 0, 1}}}; }
T4 mul(const T4& a, const T4& b) {
    T4 o{};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) for (int k = 0; k < 4; ++k) o.m[i][j] += a.m[i][k] * b.m[k][j];
    return o;
}
P3 apply(const T4& t, P3 p) {
    double o[3];
    for (int i = 0; i < 3; ++i) o[i] = t.m[i][0] * p.x + t.m[i][1] * p.y + t.m[i][2] * p.z + t.m[i][3];
    return {o[0], o[1], o[2]};
}
T4 inv4(const T4& t) {
    T4 o{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) o.m[i][j] = t.m[j][i];
    for (int i = 0; i < 3; ++i) o.m[i][3] = -(o.m[i][0] * t.m[0][3] + o.m[i][1] * t.m[1][3] + o.m[i][2] * t.m[2][3]);
    o.m[3][3] = 1.0;
    return o;
}

// ---------- 다링크 순기구학 ----------
std::vector<P2> fk(const std::vector<double>& links, const std::vector<double>& angles) {
    double x = 0, y = 0, cum = 0;
    std::vector<P2> pts{{0.0, 0.0}};
    for (size_t i = 0; i < links.size(); ++i) {
        cum += angles[i];
        x += links[i] * std::cos(rad(cum)); y += links[i] * std::sin(rad(cum));
        pts.push_back({x, y});
    }
    return pts;
}

// ---------- DH ----------
T4 dh(double a, double alpha, double d, double theta) {
    double ca = std::cos(rad(alpha)), sa = std::sin(rad(alpha)), ct = std::cos(rad(theta)), st = std::sin(rad(theta));
    return {{{ct, -st * ca, st * sa, a * ct}, {st, ct * ca, -ct * sa, a * st}, {0, sa, ca, d}, {0, 0, 0, 1}}};
}

// ---------- SAT ----------
struct OBB { double cx, cy, hw, hh, deg; };
std::vector<P2> obb_corners(const OBB& b) {
    const double sx[4] = {-1, 1, 1, -1}, sy[4] = {-1, -1, 1, 1};
    double c = std::cos(rad(b.deg)), s = std::sin(rad(b.deg));
    std::vector<P2> out;
    for (int i = 0; i < 4; ++i) { double lx = sx[i] * b.hw, ly = sy[i] * b.hh; out.push_back({b.cx + lx * c - ly * s, b.cy + lx * s + ly * c}); }
    return out;
}
void project(const std::vector<P2>& c, P2 ax, double& lo, double& hi) {
    lo = 1e300; hi = -1e300;
    for (const P2& p : c) { double v = p.x * ax.x + p.y * ax.y; if (v < lo) lo = v; if (v > hi) hi = v; }
}
}  // namespace tf

// -0.0000 방지용 출력 도우미
static void pf(double x) { if (std::fabs(x) < 5e-5) x = 0.0; std::printf("%.4f", x); }
static void p2(tf::P2 p) { std::printf("("); pf(p.x); std::printf(","); pf(p.y); std::printf(")"); }
static void p3(tf::P3 p) { std::printf("("); pf(p.x); std::printf(","); pf(p.y); std::printf(","); pf(p.z); std::printf(")"); }

static bool sat_obb(const tf::OBB& a, const tf::OBB& b) {
    using namespace tf;
    std::vector<P2> ca = obb_corners(a), cb = obb_corners(b);
    P2 axes[4] = {{std::cos(rad(a.deg)), std::sin(rad(a.deg))}, {-std::sin(rad(a.deg)), std::cos(rad(a.deg))},
                  {std::cos(rad(b.deg)), std::sin(rad(b.deg))}, {-std::sin(rad(b.deg)), std::cos(rad(b.deg))}};
    bool hit = true;
    for (int k = 0; k < 4; ++k) {
        double l1, h1, l2, h2;
        project(ca, axes[k], l1, h1); project(cb, axes[k], l2, h2);
        bool ov = !(h1 < l2 || h2 < l1);
        std::printf("  axis%d A=[", k + 1); pf(l1); std::printf(","); pf(h1); std::printf("] B=["); pf(l2); std::printf(","); pf(h2);
        std::printf("] %s\n", ov ? "overlap" : "separated");
        hit = hit && ov;
    }
    return hit;
}

int main() {
    using namespace tf;
    // ---------- A ----------
    std::printf("[A] #226 rigid2d rot=35deg t=(0.6,-0.4)\n");
    std::vector<P2> pts{{0.0, 0.0}, {1.2, 0.0}, {1.2, 0.8}, {0.0, 0.8}};
    double ang = 35.0; P2 t{0.6, -0.4};
    std::vector<P2> out = rigid2d(pts, ang, t);
    for (int i = 0; i < 4; ++i) { std::printf("P%d ", i); p2(pts[i]); std::printf(" -> "); p2(out[i]); std::printf("\n"); }
    double c = std::cos(rad(ang)), s = std::sin(rad(ang));
    std::printf("cos35="); pf(c); std::printf(" sin35="); pf(s); std::printf(" det="); pf(c * c - (-s) * s); std::printf(" area=0.9600\n");
    std::vector<P2> shifted;
    for (const P2& p : pts) shifted.push_back({p.x + t.x, p.y + t.y});
    std::printf("translate-then-rotate (wrong):");
    for (const P2& p : rigid2d(shifted, ang, {0.0, 0.0})) { std::printf(" "); p2(p); }
    std::printf("\n");
    double ia; P2 it; inv2d(ang, t, ia, it);
    std::printf("inv2d: rot="); pf(ia); std::printf("deg t'="); p2(it); std::printf("\n");
    std::printf("inv2d applied:");
    for (const P2& p : rigid2d(out, ia, it)) { std::printf(" "); p2(p); }
    std::printf("\n");
    double vx = out[1].x - t.x, vy = out[1].y - t.y;
    double by_atan2 = deg(std::atan2(vy, vx)) - deg(std::atan2(pts[1].y, pts[1].x));
    double by_acos = deg(std::acos((pts[1].x * vx + pts[1].y * vy) / (std::hypot(pts[1].x, pts[1].y) * std::hypot(vx, vy))));
    std::printf("recover_angle from P1'="); p2(out[1]); std::printf(": atan2="); pf(by_atan2); std::printf("deg acos="); pf(by_acos);
    std::printf("deg wrap(-325)="); pf(wrap(-325.0)); std::printf("deg\n");

    // ---------- B ----------
    std::printf("[B] #258 chain T_AC = T_AB * T_BC\n");
    T4 T_AB = homog_z(60.0, {0.5, 0.2, 0.0}), T_BC = homog_x(90.0, {0.0, 0.4, 0.3});
    P3 p_C{0.2, 0.1, 0.5};
    T4 T_AC = mul(T_AB, T_BC);
    std::printf("t_AC = "); p3({T_AC.m[0][3], T_AC.m[1][3], T_AC.m[2][3]}); std::printf("\n");
    std::printf("naive sum (wrong) = "); p3({0.5 + 0.0, 0.2 + 0.4, 0.0 + 0.3}); std::printf("\n");
    P3 p_B = apply(T_BC, p_C); std::printf("p_B = "); p3(p_B); std::printf("\n");
    P3 p_A = apply(T_AC, p_C); std::printf("p_A via T_AC = "); p3(p_A); std::printf("\n");
    std::printf("p_A via two steps = "); p3(apply(T_AB, p_B)); std::printf("\n");
    std::printf("reversed order T_BC*T_AB (wrong) = "); p3(apply(mul(T_BC, T_AB), p_C)); std::printf("\n");
    std::printf("inv4(T_AC) p_A = "); p3(apply(inv4(T_AC), p_A)); std::printf("\n");

    // ---------- C ----------
    std::printf("[C] #221 fk links=(1.4,0.9) angles=(25,50)\n");
    std::vector<P2> j = fk({1.4, 0.9}, {25.0, 50.0});
    for (size_t i = 0; i < j.size(); ++i) { std::printf("j%d ", (int)i); p2(j[i]); std::printf("\n"); }
    std::printf("cumulative = 25.0000/75.0000 reach = "); pf(std::hypot(j[2].x, j[2].y)); std::printf("\n");
    std::printf("absolute-angle mistake (wrong) = "); p2({j[1].x + 0.9 * std::cos(rad(50.0)), j[1].y + 0.9 * std::sin(rad(50.0))}); std::printf("\n");
    std::printf("[C] #220 fk links=(1.2,0.9,0.6) angles=(20,35,-25)\n");
    std::vector<P2> j3 = fk({1.2, 0.9, 0.6}, {20.0, 35.0, -25.0});
    for (size_t i = 0; i < j3.size(); ++i) { std::printf("j%d ", (int)i); p2(j3[i]); std::printf("\n"); }
    std::printf("cumulative = 20.0000/55.0000/30.0000\n");

    // ---------- D ----------
    std::printf("[D] #237 dh chain a=(1.4,0.9) theta=(25,50)\n");
    T4 A1 = dh(1.4, 0.0, 0.0, 25.0), A2 = dh(0.9, 0.0, 0.0, 50.0);
    std::printf("A1 R=["); pf(A1.m[0][0]); std::printf(","); pf(A1.m[0][1]); std::printf(";"); pf(A1.m[1][0]); std::printf(","); pf(A1.m[1][1]); std::printf("] t="); p2({A1.m[0][3], A1.m[1][3]}); std::printf("\n");
    std::printf("A2 R=["); pf(A2.m[0][0]); std::printf(","); pf(A2.m[0][1]); std::printf(";"); pf(A2.m[1][0]); std::printf(","); pf(A2.m[1][1]); std::printf("] t="); p2({A2.m[0][3], A2.m[1][3]}); std::printf("\n");
    T4 T = mul(A1, A2);
    std::printf("T=A1*A2 -> "); p2({T.m[0][3], T.m[1][3]}); std::printf(" yaw=atan2(T21,T11)="); pf(deg(std::atan2(T.m[1][0], T.m[0][0]))); std::printf("deg\n");
    T4 T237 = mul(dh(2.0, 0.0, 0.0, deg(0.5236)), dh(1.5, 0.0, 0.0, deg(0.7854)));
    std::printf("original #237 a=(2.0,1.5) theta=(0.5236,0.7854)rad -> "); p2({T237.m[0][3], T237.m[1][3]}); std::printf("\n");

    // ---------- E ----------
    std::printf("[E] #223 sweep theta1=25 theta2=0..90 step 15\n");
    int n = 0;
    for (int a = 0; a <= 90; a += 15) {
        P2 e = fk({1.4, 0.9}, {25.0, (double)a})[2];
        std::printf("%02d ", a); p2(e); std::printf(" r_from_j1="); pf(std::hypot(e.x - j[1].x, e.y - j[1].y));
        std::printf(" dist="); pf(std::hypot(e.x, e.y)); std::printf("\n");
        ++n;
    }
    std::printf("points = %d center j1 = ", n); p2(j[1]); std::printf(" annulus inner=0.5000 outer=2.3000\n");

    // ---------- #512 ----------
    std::printf("[#512] SAT obb A=(0,0,0.6,0.4,0deg) B=(1.1,0.35,0.5,0.3,30deg)\n");
    OBB A{0.0, 0.0, 0.6, 0.4, 0.0};
    for (double cx : {1.1, 1.45}) {
        OBB B{cx, 0.35, 0.5, 0.3, 30.0};
        std::printf("B cx="); pf(cx); std::printf(" corners:");
        for (const P2& p : obb_corners(B)) { std::printf(" "); p2(p); }
        std::printf("\n");
        std::printf("  collide = %s\n", sat_obb(A, B) ? "True" : "False");
    }

    // ---------- #558 ----------
    std::printf("[#558] circular mean headings=(10,350,20)\n");
    std::vector<double> h{10.0, 350.0, 20.0};
    std::printf("simple mean (wrong) = "); pf((h[0] + h[1] + h[2]) / 3.0); std::printf("\n");
    std::printf("circular mean = "); pf(circular_mean(h)); std::printf("deg\n");
    return 0;
}
