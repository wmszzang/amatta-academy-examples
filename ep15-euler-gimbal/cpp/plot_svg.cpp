#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

int main() {
    std::ifstream input("euler.csv");
    if (!input) { std::cerr << "Run euler_gimbal first.\n"; return 1; }
    std::string line;
    std::getline(input, line);
    std::vector<std::vector<double> > rows;
    while (std::getline(input, line)) {
        std::stringstream fields(line);
        std::string field;
        std::getline(fields, field, ',');
        std::vector<double> values;
        while (std::getline(fields, field, ',')) values.push_back(std::stod(field));
        if (values.size() != 3) return 2;
        rows.push_back(values);
    }
    if (rows.empty()) return 2;
    std::ofstream output("euler.svg");
    if (!output) return 3;
    output << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
           << "<rect width='1200' height='800' fill='#F4F6FB'/>"
           << "<g font-family='sans-serif' fill='#1E293B'>"
           << "<text x='70' y='75' font-size='36'>ZYX decomposition (rad)</text>";
    const char* labels[] = {"roll", "pitch", "yaw"};
    const char* colors[] = {"#E11D48", "#059669", "#4F46E5"};
    // 세 각의 부호와 크기를 같은 축척으로 비교한다.
    for (std::size_t row=0; row<rows.size(); ++row) {
        const double top=140+row*(560.0/rows.size());
        output << "<text x='70' y='" << top << "' font-size='24'>case " << row+1 << "</text>";
        for (int col=0; col<3; ++col) {
            const double y=top+25+col*35;
            output << "<text x='180' y='" << y+6 << "' font-size='20'>" << labels[col] << "</text>"
                   << "<line x1='650' y1='" << y << "' x2='" << 650+rows[row][col]*140
                   << "' y2='" << y << "' stroke='" << colors[col] << "' stroke-width='16'/>"
                   << "<text x='990' y='" << y+6 << "' font-size='20'>" << rows[row][col] << "</text>";
        }
    }
    output << "</g></svg>";
    std::cout << "euler.svg\n";
}
