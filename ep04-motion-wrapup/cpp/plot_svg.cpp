// 네 지문의 CSV → motion_wrapup.svg (라이브러리 없이 SVG 직접 쓰기). C++ 버전 시각화.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.4
//
// 컴파일·실행 (motion_wrapup.exe 를 먼저 돌려 CSV 4개를 만든 뒤, 같은 폴더에서):
//     cl /EHsc plot_svg.cpp        (또는 g++ -std=c++11 plot_svg.cpp -o plot_svg)
//     plot_svg.exe                 → motion_wrapup.svg  (브라우저로 열면 2×2 속도 프로파일)
//
// 패널 4개: 지문 ① 사다리꼴 · ② 3차(종 모양) · ③ 5차 · ④ S-커브. 각 패널에 속도(굵은 선)와 위치(가는 회색 선, 자기 최댓값으로 정규화).
#include <cstdio>
#include <string>
#include <vector>

struct Series { std::vector<double> t, x, v; };

static bool load(const char* name, Series& s) {
    std::FILE* f = std::fopen(name, "r");
    if (f == nullptr) { std::printf("cannot open %s (motion_wrapup 을 먼저 실행하세요)\n", name); return false; }
    char line[256];
    std::fgets(line, sizeof line, f);                      // 헤더 건너뛰기
    while (std::fgets(line, sizeof line, f)) {
        double t, x, v;
        if (std::sscanf(line, "%lf,%lf,%lf", &t, &x, &v) == 3) { s.t.push_back(t); s.x.push_back(x); s.v.push_back(v); }
    }
    std::fclose(f);
    return !s.t.empty();
}

static double vmax(const std::vector<double>& y) { double m = 0; for (double a : y) if (a > m) m = a; return m; }

// 점들을 선으로 이어 적는 함수 — 패널 좌표계(px, py, w, h)에 맞춰 스케일.
static void polyline(std::FILE* f, const Series& s, const std::vector<double>& Y, double ymax,
                     double px, double py, double w, double h, const char* color, double width) {
    double tmax = s.t.back();
    std::fprintf(f, "<polyline fill='none' stroke='%s' stroke-width='%.1f' points='", color, width);
    for (std::size_t i = 0; i < s.t.size(); ++i)
        std::fprintf(f, "%.1f,%.1f ", px + s.t[i] / tmax * w, py + h - Y[i] / ymax * h * 0.9);
    std::fprintf(f, "'/>\n");
}

int main() {
    const char* files[4] = {"p1_trapezoid.csv", "p2_cubic.csv", "p3_quintic.csv", "p4_scurve.csv"};
    const char* titles[4] = {"Prompt 1  trapezoid", "Prompt 2  cubic", "Prompt 3  quintic", "Prompt 4  S-curve"};
    const char* colors[4] = {"#0EA5E9", "#4F46E5", "#10B981", "#F59E0B"};
    Series s[4];
    for (int i = 0; i < 4; ++i) if (!load(files[i], s[i])) return 1;

    std::FILE* f = std::fopen("motion_wrapup.svg", "w");
    if (f == nullptr) { std::printf("cannot open motion_wrapup.svg\n"); return 1; }
    std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='1280' height='720' font-family='sans-serif' font-size='16'>\n");
    std::fprintf(f, "<rect width='1280' height='720' fill='white'/>\n");
    const double W = 540, H = 260;
    for (int i = 0; i < 4; ++i) {
        double px = 80 + (i % 2) * 620, py = 60 + (i / 2) * 340;
        std::fprintf(f, "<rect x='%.0f' y='%.0f' width='%.0f' height='%.0f' fill='none' stroke='#999'/>\n", px, py, W, H);
        std::fprintf(f, "<text x='%.0f' y='%.0f' font-weight='bold'>%s</text>\n", px, py - 12, titles[i]);
        std::fprintf(f, "<text x='%.0f' y='%.0f' fill='#666'>time (s)  ->  %.2f</text>\n", px + W - 150, py + H + 20, s[i].t.back());
        std::fprintf(f, "<text x='%.0f' y='%.0f' fill='#666'>velocity max %.2f</text>\n", px, py + H + 20, vmax(s[i].v));
        polyline(f, s[i], s[i].x, vmax(s[i].x), px, py, W, H, "#BBBBBB", 1.5);   // 위치(정규화, 회색)
        polyline(f, s[i], s[i].v, vmax(s[i].v), px, py, W, H, colors[i], 3.0);   // 속도(굵게)
    }
    std::fprintf(f, "</svg>\n");
    std::fclose(f);
    std::printf("saved: motion_wrapup.svg (open it in a browser)\n");
    return 0;
}
