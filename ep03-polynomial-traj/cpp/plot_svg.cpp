// cubic.csv · quintic.csv 를 읽어 trajectory.svg(각도·각속도·각가속도 3단, 3차 vs 5차)로 그린다.
// C++ 시각화, 별도 라이브러리 없음.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.3
//
// 어떤 언어(C/C++/Python)가 만든 CSV든 읽을 수 있습니다.
// 결과 trajectory.svg 는 브라우저로 열면 바로 보입니다.
// (시험장이라면 같은 CSV를 Excel로 열어 꺾은선 차트를 그려도 됩니다.)
//
// 컴파일·실행 (g++ / MinGW):
//     g++ -std=c++11 plot_svg.cpp -o plot_svg
//     ./plot_svg             (Windows: plot_svg.exe)
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl /EHsc plot_svg.cpp
//     plot_svg.exe
#include <cstdio>
#include <vector>

struct Series { std::vector<double> t, q, qd, qdd; };

static bool load(const char* path, Series& s) {
    std::FILE* in = std::fopen(path, "r");
    if (in == nullptr) { std::printf("%s not found - run the calculation program first.\n", path); return false; }
    char header[64];
    std::fgets(header, sizeof header, in);                 // "t,q,qd,qdd" 헤더 건너뛰기
    double t, q, qd, qdd;
    while (std::fscanf(in, "%lf,%lf,%lf,%lf", &t, &q, &qd, &qdd) == 4) {
        s.t.push_back(t); s.q.push_back(q); s.qd.push_back(qd); s.qdd.push_back(qdd);
    }
    std::fclose(in);
    return !s.t.empty();
}

// 한 패널(가로 밴드)에 폴리라인 하나를 쓴다. y는 [lo, hi]를 패널 높이에 맞춰 뒤집는다.
static void polyline(std::FILE* f, const std::vector<double>& T, const std::vector<double>& Y,
                     double tmax, double lo, double hi, double top, double h, const char* color) {
    std::fprintf(f, "<polyline fill='none' stroke='%s' stroke-width='3' points='", color);
    for (std::size_t i = 0; i < T.size(); ++i)
        std::fprintf(f, "%.1f,%.1f ", 70 + T[i] / tmax * 860, top + (hi - Y[i]) / (hi - lo) * h);
    std::fprintf(f, "'/>\n");
}

int main() {
    Series c, q;
    if (!load("cubic.csv", c) || !load("quintic.csv", q)) return 1;
    double tmax = c.t.back();
    const std::vector<double>* cols[3][2] = {{&c.q, &q.q}, {&c.qd, &q.qd}, {&c.qdd, &q.qdd}};
    const char* names[3] = {"angle (deg)", "velocity (deg/s)", "accel (deg/s^2)"};

    std::FILE* f = std::fopen("trajectory.svg", "w");
    std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='960' height='540' "
                    "style='background:#F7F8FC;font-family:sans-serif'>\n");
    for (int p = 0; p < 3; ++p) {
        double lo = 1e18, hi = -1e18;                       // 두 곡선의 공통 y 범위
        for (int k = 0; k < 2; ++k)
            for (double v : *cols[p][k]) { if (v < lo) lo = v; if (v > hi) hi = v; }
        double pad = (hi - lo) * 0.12; lo -= pad; hi += pad;
        double top = 20 + p * 170, h = 140;
        std::fprintf(f, "<rect x='70' y='%.0f' width='860' height='%.0f' fill='white' stroke='#CBD5E1'/>\n", top, h);
        if (lo < 0 && hi > 0) {                             // 0 기준선
            double y0 = top + hi / (hi - lo) * h;
            std::fprintf(f, "<line x1='70' y1='%.1f' x2='930' y2='%.1f' stroke='#94A3B8'/>\n", y0, y0);
        }
        std::fprintf(f, "<text x='75' y='%.0f' font-size='13' fill='#334155'>%s</text>\n", top + 16, names[p]);
        polyline(f, c.t, *cols[p][0], tmax, lo, hi, top, h, "#4F46E5");   // 3차 = 인디고
        polyline(f, q.t, *cols[p][1], tmax, lo, hi, top, h, "#059669");   // 5차 = 에메랄드
    }
    std::fprintf(f, "<text x='500' y='532' font-size='13' fill='#334155' text-anchor='middle'>time (s)  |  "
                    "<tspan fill='#4F46E5'>cubic</tspan>  <tspan fill='#059669'>quintic</tspan></text>\n");
    std::fprintf(f, "</svg>\n");
    std::fclose(f);
    std::printf("saved: trajectory.svg (open it in a browser)\n");
    return 0;
}
