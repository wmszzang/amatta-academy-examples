// points.csv (x,y,xr,yr) → frames.svg (라이브러리 없이 SVG 직접 쓰기). C++ 버전 시각화.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.5
//
// 컴파일·실행 (frames2d.exe 를 먼저 돌려 points.csv 를 만든 뒤, 같은 폴더에서):
//     cl /EHsc plot_svg.cpp        (또는 g++ -std=c++11 plot_svg.cpp -o plot_svg)
//     plot_svg.exe                 → frames.svg  (브라우저로 열면 원본 정사각형(회색 점선)과 변환 후(인디고))
#include <cstdio>
#include <vector>

struct Pt { double x, y; };

static bool load(const char* name, std::vector<Pt>& orig, std::vector<Pt>& moved) {
    std::FILE* f = std::fopen(name, "r");
    if (f == nullptr) { std::printf("cannot open %s (frames2d 를 먼저 실행하세요)\n", name); return false; }
    char line[256];
    std::fgets(line, sizeof line, f);                      // 헤더 건너뛰기
    while (std::fgets(line, sizeof line, f)) {
        double x, y, xr, yr;
        if (std::sscanf(line, "%lf,%lf,%lf,%lf", &x, &y, &xr, &yr) == 4) { orig.push_back({x, y}); moved.push_back({xr, yr}); }
    }
    std::fclose(f);
    return !orig.empty();
}

// 데이터 좌표(−0.3~2.2, equal aspect) → 화면 좌표. 화면 높이 720 을 데이터 2.5 에 대응.
static const double X0 = -0.3, X1 = 2.2, Y0 = -0.3, Y1 = 2.2;
static const double PX = 380, PY = 40, PW = 640, PH = 640;
static double sx(double x) { return PX + (x - X0) / (X1 - X0) * PW; }
static double sy(double y) { return PY + PH - (y - Y0) / (Y1 - Y0) * PH; }

static void polygon(std::FILE* f, const std::vector<Pt>& p, const char* color, double width, const char* dash) {
    std::fprintf(f, "<polygon fill='none' stroke='%s' stroke-width='%.1f'%s points='", color, width, dash);
    for (const Pt& q : p) std::fprintf(f, "%.1f,%.1f ", sx(q.x), sy(q.y));
    std::fprintf(f, "'/>\n");
    for (const Pt& q : p) std::fprintf(f, "<circle cx='%.1f' cy='%.1f' r='5' fill='%s'/>\n", sx(q.x), sy(q.y), color);
}

int main() {
    std::vector<Pt> orig, moved;
    if (!load("points.csv", orig, moved)) return 1;

    std::FILE* f = std::fopen("frames.svg", "w");
    if (f == nullptr) { std::printf("cannot open frames.svg\n"); return 1; }
    std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='1280' height='720' font-family='sans-serif' font-size='18'>\n");
    std::fprintf(f, "<rect width='1280' height='720' fill='white'/>\n");
    // 격자 + 축
    for (int i = 0; i <= 4; ++i) {
        double v = i * 0.5;
        std::fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#DDD' stroke-dasharray='4 4'/>\n", sx(v), sy(Y0), sx(v), sy(Y1));
        std::fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#DDD' stroke-dasharray='4 4'/>\n", sx(X0), sy(v), sx(X1), sy(v));
        std::fprintf(f, "<text x='%.1f' y='%.1f' fill='#666' text-anchor='middle'>%.1f</text>\n", sx(v), sy(Y0) + 24, v);
        std::fprintf(f, "<text x='%.1f' y='%.1f' fill='#666' text-anchor='end'>%.1f</text>\n", sx(X0) - 8, sy(v) + 6, v);
    }
    std::fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#1E293B' stroke-width='2'/>\n", sx(X0), sy(0), sx(X1), sy(0));
    std::fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#1E293B' stroke-width='2'/>\n", sx(0), sy(Y0), sx(0), sy(Y1));
    polygon(f, orig, "#9CA3AF", 2.0, " stroke-dasharray='8 6'");
    polygon(f, moved, "#4F46E5", 3.5, "");
    for (const Pt& q : moved)
        std::fprintf(f, "<text x='%.1f' y='%.1f' fill='#4F46E5'>(%.3f, %.3f)</text>\n", sx(q.x) + 8, sy(q.y) - 8, q.x, q.y);
    std::fprintf(f, "<text x='40' y='60' font-weight='bold' font-size='24'>2D rigid transform</text>\n");
    std::fprintf(f, "<text x='40' y='96' fill='#666'>rotate 30 deg, then move (1, 0.5)</text>\n");
    std::fprintf(f, "<text x='40' y='140' fill='#9CA3AF'>--- original</text>\n");
    std::fprintf(f, "<text x='40' y='170' fill='#4F46E5'>--- transformed</text>\n");
    std::fprintf(f, "</svg>\n");
    std::fclose(f);
    std::printf("saved: frames.svg (open it in a browser)\n");
    return 0;
}
