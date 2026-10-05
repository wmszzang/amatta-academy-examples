// 입력 좌표를 같은 비율로 옮겨 선분과 접촉 원을 SVG로 쓴다.
#include <fstream>
int main(){
    std::ofstream f("result.svg");
    if(!f)return 1;
    f<<R"(<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="800" viewBox="0 0 1200 800">
<rect width="1200" height="800" fill="#F4F6FB"/>
<g font-family="Arial" fill="#1E293B" font-size="30">
<text x="80" y="80">EP.23: segment-circle contact</text>
<path d="M 180 600 H 1020 M 300 700 V 130" fill="none" stroke="#94A3B8" stroke-width="2"/>
<path d="M 300 600 H 860" stroke="#4F46E5" stroke-width="8"/>
<circle cx="580" cy="460" r="140" fill="#FFE4E6" stroke="#E11D48" stroke-width="4"/>
<path d="M580 460 V600" stroke="#E11D48" stroke-dasharray="8 8" stroke-width="4"/>
<circle cx="580" cy="460" r="7" fill="#E11D48"/>
<text x="190" y="650">A=(0,0)</text><text x="810" y="650">B=(4,0)</text>
<text x="610" y="460">C=(2,1)</text><text x="610" y="550">d=1, r=1</text>
<text x="390" y="210">d &lt;= r: COLLISION</text>
<text x="80" y="755">Equal x/y scale; contact counts as collision.</text>
</g></svg>)";
}
