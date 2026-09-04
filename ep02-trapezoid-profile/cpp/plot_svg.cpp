// profile.csv를 읽어 profile.svg(속도·거리 두 그래프)로 그린다 — C++ 시각화, 별도 라이브러리 없음.
//
// Amatta Academy | 자격증 · 로봇소프트웨어개발기사 · 실기 EP.2
//
// 어떤 언어(C/C++/Python)가 만든 profile.csv든 읽을 수 있습니다.
// 결과 profile.svg는 브라우저로 열면 바로 보입니다.
// (시험장이라면 같은 CSV를 Excel로 열어 꺾은선 차트를 그려도 됩니다.)
//
// 컴파일·실행 (g++ / MinGW):
//     g++ -std=c++11 plot_svg.cpp -o plot_svg
//     ./plot_svg             (Windows: plot_svg.exe)
//
// 컴파일·실행 (시험장 Visual Studio 개발자 명령 프롬프트):
//     cl /EHsc plot_svg.cpp
//     plot_svg.exe
#include <cstdio>
#include <vector>

int main() {
    std::FILE* in = std::fopen("profile.csv", "r");
    if (in == nullptr) {
        std::printf("profile.csv가 없습니다. 계산 프로그램을 먼저 실행하세요.\n");
        return 1;
    }
    char header[64];
    std::fgets(header, sizeof header, in);          // "t,s,v" 헤더 건너뛰기
    std::vector<double> T, S, V;
    double t, s, v;
    while (std::fscanf(in, "%lf,%lf,%lf", &t, &s, &v) == 3) {
        T.push_back(t); S.push_back(s); V.push_back(v);
    }
    std::fclose(in);
    if (T.empty()) { std::printf("profile.csv에 데이터가 없습니다.\n"); return 1; }

    double tmax = T.back(), smax = 0.0, vmax = 0.0;
    for (double x : S) if (x > smax) smax = x;
    for (double x : V) if (x > vmax) vmax = x;

    std::FILE* f = std::fopen("profile.svg", "w");
    std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='960' height='540'>\n");
    std::fprintf(f, "<rect width='960' height='540' fill='#F7F8FC'/>\n");
    std::fprintf(f, "<line x1='60' y1='240' x2='920' y2='240' stroke='#94A3B8' stroke-width='2'/>\n");
    std::fprintf(f, "<line x1='60' y1='510' x2='920' y2='510' stroke='#94A3B8' stroke-width='2'/>\n");
    std::fprintf(f, "<polyline fill='none' stroke='#4F46E5' stroke-width='3' points='");
    for (std::size_t i = 0; i < T.size(); ++i)      // 위쪽: 속도-시간
        std::fprintf(f, "%.1f,%.1f ", 60.0 + T[i] / tmax * 860.0, 240.0 - V[i] / vmax * 200.0);
    std::fprintf(f, "'/>\n<polyline fill='none' stroke='#059669' stroke-width='3' points='");
    for (std::size_t i = 0; i < T.size(); ++i)      // 아래쪽: 거리-시간
        std::fprintf(f, "%.1f,%.1f ", 60.0 + T[i] / tmax * 860.0, 510.0 - S[i] / smax * 200.0);
    std::fprintf(f, "'/>\n</svg>\n");
    std::fclose(f);
    std::printf("profile.svg 저장 완료 — 브라우저로 열어 보세요.\n");
    return 0;
}
