// CSV의 실제 계산 결과를 같은 축척의 SVG로 변환한다.
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
int main(int argc,char**argv) {
    std::ifstream in(argc>1?argv[1]:"compare.csv");
    if(!in){std::cerr<<"cannot open CSV\n";return 2;}
    std::string line;std::getline(in,line);
    std::vector<double> xj,yj,xc,yc;
    while(std::getline(in,line)) {
        std::stringstream ss(line);std::string cell;std::vector<double> r;
        while(std::getline(ss,cell,','))r.push_back(std::stod(cell));
        if(r.size()!=10){std::cerr<<"invalid row\n";return 3;}
        xj.push_back(r[3]);yj.push_back(r[4]);xc.push_back(r[6]);yc.push_back(r[7]);
    }
    if(xj.empty())return 3;
    std::ofstream out("csv-trajectory.svg");
    out<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800'><rect width='1200' height='800' fill='#f4f6fb'/><text x='80' y='60' font-size='32'>EP.19: sampled joint / Cartesian paths</text>";
    for(int k=0;k<2;k++) {
        out<<"<polyline fill='none' stroke='"<<(k?"#059669":"#4f46e5")<<"' stroke-width='5' points='";
        for(size_t i=0;i<xj.size();i++)out<<100+(k?xc[i]:xj[i])*350<<","<<700-(k?yc[i]:yj[i])*350<<" ";
        out<<"'/>";
    }
    out<<"<text x='80' y='760' font-size='24'>Equal x/y scale (m); 11 samples. Dense length is computed separately.</text></svg>";
}
