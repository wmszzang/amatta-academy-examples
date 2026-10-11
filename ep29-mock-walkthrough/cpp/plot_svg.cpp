// 계산 프로그램이 저장한 CSV를 읽어 SVG를 직접 쓴다. 좌표축 배율은 같다.
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
std::vector<std::vector<double>> read_csv(const char* path,bool skip_id=false) {
    std::ifstream file(path); if(!file) throw std::runtime_error(path);
    std::string line,cell;std::getline(file,line);
    std::vector<std::vector<double>> result;
    while(std::getline(file,line)) {
        std::stringstream stream(line);std::vector<double> row;
        if(skip_id) std::getline(stream,cell,',');
        while(std::getline(stream,cell,',')) row.push_back(std::stod(cell));
        result.push_back(row);
    }
    return result;
}
int main() {
    try {
        auto rows=read_csv("trajectory.csv"),points=read_csv("points.csv",true);
        std::ofstream svg("result.svg");if(!svg)return 1;
        svg<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'><rect width='1200' height='800' fill='#f4f6fb'/><g transform='translate(130,0) scale(0.78)' font-family='Arial' fill='#172033'><text x='70' y='55' font-size='27'>EP.29: actual CSV results</text>";
        const char* labels[]={"x (m)","v (m/s)","a (m/s²)"};
        double maxima[]={10,2,1};
        for(int k=0;k<3;++k){
            double top=120+k*180;
            svg<<"<text x='65' y='"<<top-15<<"' font-size='20'>"<<labels[k]<<"</text><path d='M 100 "<<top<<" v 130 h 500' stroke='#94a3b8' fill='none'/><polyline stroke='#4f46e5' stroke-width='3' fill='none' points='";
            for(size_t i=0;i<rows.size();++i){
                double x=100+rows[i][0]*500/7;
                auto py=[&](double value){return top+120-(k==2?(value+1)*55:value*110/maxima[k]);};
                if(k==2 && i>0) svg<<x<<','<<py(rows[i-1][k+1])<<' ';
                svg<<x<<','<<py(rows[i][k+1])<<' ';
            }
            svg<<"'/><text x='280' y='"<<top+155<<"' font-size='18'>t (s): 0 to 7</text>";
        }
        svg<<"<text x='710' y='105' font-size='22'>Original / transformed square</text><path d='M 710 600 H 1140 M 750 650 V 150' stroke='#94a3b8' fill='none'/>";
        for(int offset:{0,2}){
            svg<<"<polyline fill='none' stroke='"<<(offset==0?"#94a3b8":"#4f46e5")<<"' stroke-width='4' points='";
            for(size_t i=0;i<=points.size();++i){auto p=points[i%points.size()];svg<<750+180*p[offset]<<','<<600-180*p[offset+1]<<' ';}
            svg<<"'/>";
        }
        svg<<"<text x='1080' y='630' font-size='22'>x</text><text x='720' y='160' font-size='22'>y</text><text x='720' y='700' font-size='20'>Equal units on both axes</text></g></svg>";
        std::cout<<"result.svg written from trajectory.csv and points.csv\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
