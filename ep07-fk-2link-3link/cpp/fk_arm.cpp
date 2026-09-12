#include <cmath>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

using Pt = std::pair<double, double>;
const double PI = 3.14159265358979323846;

std::vector<Pt> fk(const std::vector<double>& theta, const std::vector<double>& length) {
    std::vector<Pt> points{{0.0, 0.0}};
    double acc = 0.0, x = 0.0, y = 0.0;
    for (size_t i = 0; i < theta.size(); ++i) {
        acc += theta[i] * PI / 180.0;
        x += length[i] * std::cos(acc); y += length[i] * std::sin(acc);
        points.push_back({x, y});
    }
    return points;
}

int main() {
    for (const auto& point : fk({30, 45}, {1.5, 1.0}))
        std::cout << std::fixed << std::setprecision(4) << "(" << point.first << "," << point.second << ") ";
    std::cout << "\n";
}
