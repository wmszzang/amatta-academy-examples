// EP.13 lambda 스윕 곡선을 SVG로 직접 출력한다(그래픽 라이브러리 없음).
// 빌드: g++ -std=c++11 cpp/plot_svg.cpp -o plot_svg.exe
// 실행: ./plot_svg.exe lam_sweep.svg
#include <cstdio>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace {
const double TOL = 1e-6;
const int ITMAX = 200;

int dlsUpdates(double gx, double gy, double t1, double t2,
               double l1, double l2, double lam) {
    for (int it = 0; it < ITMAX; ++it) {
        double px = l1 * std::cos(t1) + l2 * std::cos(t1 + t2);
        double py = l1 * std::sin(t1) + l2 * std::sin(t1 + t2);
        double ex = gx - px, ey = gy - py;
        if (std::sqrt(ex * ex + ey * ey) < TOL) return it;
        double s1 = std::sin(t1), c1 = std::cos(t1);
        double s12 = std::sin(t1 + t2), c12 = std::cos(t1 + t2);
        double J00 = -l1 * s1 - l2 * s12, J01 = -l2 * s12;
        double J10 =  l1 * c1 + l2 * c12, J11 =  l2 * c12;
        double a11 = J00 * J00 + J01 * J01 + lam * lam;
        double a12 = J00 * J10 + J01 * J11;
        double a22 = J10 * J10 + J11 * J11 + lam * lam;
        double det = a11 * a22 - a12 * a12;
        double u = ( a22 * ex - a12 * ey) / det;
        double v = (-a12 * ex + a11 * ey) / det;
        t1 += J00 * u + J10 * v;
        t2 += J01 * u + J11 * v;
    }
    return ITMAX;
}
}  // namespace

int main(int argc, char** argv) {
    std::string out = argc > 1 ? argv[1] : "lam_sweep.svg";
    const double lams[5] = {0.01, 0.05, 0.1, 0.3, 1.0};
    const char* labels[5] = {"0.01", "0.05", "0.1", "0.3", "1.0"};
    std::vector<int> counts;
    for (int k = 0; k < 5; ++k) counts.push_back(dlsUpdates(3.4, 0.15, 0.0, 0.0, 2.0, 1.5, lams[k]));

    const int W = 960, H = 540, L = 90, R = 40, T = 70, B = 70;
    const double plotW = W - L - R, plotH = H - T - B;
    std::ofstream f(out.c_str());
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << W << "\" height=\"" << H
      << "\" viewBox=\"0 0 " << W << " " << H << "\">\n";
    f << "<rect width=\"" << W << "\" height=\"" << H << "\" fill=\"#F4F6FB\"/>\n";
    f << "<text x=\"" << W / 2 << "\" y=\"38\" font-family=\"sans-serif\" font-size=\"26\""
      << " fill=\"#1E293B\" text-anchor=\"middle\">DLS lambda sweep - updates to converge</text>\n";
    for (int g = 0; g <= 5; ++g) {
        double val = g * 30.0;
        double y = T + plotH - plotH * val / 150.0;
        f << "<line x1=\"" << L << "\" y1=\"" << y << "\" x2=\"" << (L + plotW) << "\" y2=\"" << y
          << "\" stroke=\"#CBD5E1\" stroke-width=\"1\"/>\n";
        f << "<text x=\"" << (L - 12) << "\" y=\"" << (y + 5)
          << "\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#64748B\" text-anchor=\"end\">"
          << (int)val << "</text>\n";
    }
    double slot = plotW / 5.0, bw = slot * 0.52;
    for (int k = 0; k < 5; ++k) {
        double h = plotH * counts[k] / 150.0;
        double x = L + slot * k + (slot - bw) / 2.0;
        double y = T + plotH - h;
        const char* color = counts[k] <= 10 ? "#059669" : "#D97706";
        f << "<rect x=\"" << x << "\" y=\"" << y << "\" width=\"" << bw << "\" height=\"" << h
          << "\" fill=\"" << color << "\" rx=\"5\"/>\n";
        f << "<text x=\"" << (x + bw / 2) << "\" y=\"" << (y - 10)
          << "\" font-family=\"sans-serif\" font-size=\"19\" fill=\"#1E293B\" text-anchor=\"middle\">"
          << counts[k] << "</text>\n";
        f << "<text x=\"" << (x + bw / 2) << "\" y=\"" << (T + plotH + 26)
          << "\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1E293B\" text-anchor=\"middle\">"
          << labels[k] << "</text>\n";
    }
    f << "<line x1=\"" << L << "\" y1=\"" << (T + plotH) << "\" x2=\"" << (L + plotW)
      << "\" y2=\"" << (T + plotH) << "\" stroke=\"#1E293B\" stroke-width=\"2\"/>\n";
    f << "<text x=\"" << (L + plotW / 2) << "\" y=\"" << (H - 18)
      << "\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1E293B\" text-anchor=\"middle\">lambda</text>\n";
    f << "</svg>\n";
    f.close();
    std::printf("wrote %s  (updates:", out.c_str());
    for (int k = 0; k < 5; ++k) std::printf(" %d", counts[k]);
    std::printf(")\n");
    return 0;
}
