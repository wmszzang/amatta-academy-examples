// 실행된 오도메트리 CSV의 좌표를 같은 축척으로 SVG에 옮긴다.
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
struct Point { double x,y; };
int main(int argc,char** argv) {
    if(argc!=3){std::cerr<<"usage: plot_svg odom.csv result.svg\n";return 1;}
    std::ifstream in(argv[1]);if(!in)return 2;
    std::string line;std::getline(in,line);std::vector<Point> points;
    while(std::getline(in,line)) {
        std::stringstream row(line);std::string i,x,y;
        std::getline(row,i,',');std::getline(row,x,',');std::getline(row,y,',');
        points.push_back({std::stod(x),std::stod(y)});
    }
    std::ofstream out(argv[2]);if(!out||points.empty())return 3;
    out<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'><rect width='1200' height='800' fill='#f4f6fb'/><g font-family='Arial' fill='#172033' font-size='28'><text x='70' y='60'>Encoder odometry - midpoint integration</text><text x='900' y='690'>x (m)</text><text x='35' y='135'>y (m)</text><text x='70' y='750'>Same x/y scale: 600 pixels per metre</text></g><path d='M100 100 V650 H1050' fill='none' stroke='#172033' stroke-width='3'/><polyline fill='none' stroke='#4f46e5' stroke-width='6' points='";
    for(const auto&p:points)out<<100+600*p.x<<","<<650-600*p.y<<" ";
    out<<"'/></svg>";return 0;
}
