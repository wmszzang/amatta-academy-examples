#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

// 계산 프로그램이 만든 CSV를 읽으므로 그래프용 수치를 따로 꾸미지 않는다.
std::vector<std::vector<double> > read_csv(const char* file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error(std::string("Missing CSV: ")+file);
    std::string line, cell;
    std::getline(in,line);
    std::vector<std::vector<double> > result;
    while(std::getline(in,line)) {
        std::stringstream row(line);
        std::vector<double> values;
        while(std::getline(row,cell,',')) values.push_back(std::stod(cell));
        result.push_back(values);
    }
    return result;
}

void plot(std::ofstream& out,const std::vector<std::vector<double> >& rows,int xc,int yc,
          double left,double top,double xmax,double ymax,const char* color) {
    out << "<polyline fill='none' stroke='" << color << "' stroke-width='3' points='";
    for(const auto& r:rows) out << left+r[xc]/xmax*460 << ',' << top+270-r[yc]/ymax*270 << ' ';
    out << "'/>\n";
}

int main() {
    auto plant=read_csv("plant.csv"), angles=read_csv("complementary.csv");
    std::ofstream out("control-wrapup.svg");
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
        << "<rect width='1200' height='800' fill='#f4f6fb'/>"
        << "<g font-family='sans-serif' fill='#1e293b'>"
        << "<text x='70' y='100' font-size='32'>EP.28: computed CSV results</text>"
        << "<text x='70' y='190' font-size='24'>#245 Plant output / time (s)</text>"
        << "<text x='660' y='190' font-size='24'>#228 Angle (deg) / time (s)</text>"
        << "<path d='M70 220V490H530 M660 220V490H1120' fill='none' stroke='#1e293b'/>"
        << "<text x='70' y='525'>0</text><text x='510' y='525'>10 s</text>"
        << "<text x='660' y='525'>0</text><text x='1100' y='525'>2 s</text>"
        << "<text x='70' y='595' font-size='22'>Plant: 0.993958 (target 1)</text>"
        << "<text x='660' y='595' font-size='22'>Gyro 24 / filter 22.437753 / truth 20</text></g>";
    plot(out,plant,3,4,70,220,10,1.1,"#4f46e5");
    plot(out,angles,1,5,660,220,2,25,"#e11d48");
    plot(out,angles,1,3,660,220,2,25,"#0284c7");
    plot(out,angles,1,4,660,220,2,25,"#4f46e5");
    out << "</svg>";
    std::cout << "control-wrapup.svg: from plant.csv and complementary.csv\n";
}
