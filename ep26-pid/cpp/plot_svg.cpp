#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>

int main() {
    std::ifstream in("position.csv");
    if(!in) { std::cerr << "Run pid first\n"; return 1; }
    std::string line; std::getline(in,line);
    std::vector<double> ts,xs,us;
    while(std::getline(in,line)) {
        std::stringstream row(line); std::string cell; std::vector<double> v;
        while(std::getline(row,cell,',')) v.push_back(std::stod(cell));
        if(v.size()!=8) return 1;
        ts.push_back(v[0]); xs.push_back(v[1]); us.push_back(v[7]);
    }
    if(ts.size()!=31) return 1;
    std::ofstream out("pid.svg");
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='650' viewBox='0 0 1200 650'><rect width='1200' height='650' fill='#F4F6FB'/>";
    out << "<g font-family='sans-serif' fill='#1E293B' font-size='24'><text x='90' y='40'>C++ / #229 PID: same calculated CSV</text>";
    out << "<text x='90' y='78'>Position (problem scale)</text><text x='90' y='353'>Control input</text><text x='520' y='625'>Time (s): 0 to 3</text></g>";
    for(int j=0;j<2;j++) {
        double top=j?380:100, h=200;
        out << "<path d='M90," << top << " V" << top+h << " H1120' fill='none' stroke='#64748B' stroke-width='2'/>";
        if(!j) out << "<path d='M90,133.333 H1120' stroke='#D97706' stroke-dasharray='8 8'/><text x='975' y='120' font-family='sans-serif' font-size='20'>target 1</text>";
        out << "<polyline fill='none' stroke='" << (j?"#0284C7":"#4F46E5") << "' stroke-width='4' points='";
        for(size_t k=0;k<ts.size();k++) out << 90+1030*ts[k]/3 << "," << top+h-(j?us[k]/2.2:xs[k]/1.2)*h << " ";
        out << "'/>";
    }
    out << "</svg>";
    std::cout << "pid.svg: 31 samples\n";
}
