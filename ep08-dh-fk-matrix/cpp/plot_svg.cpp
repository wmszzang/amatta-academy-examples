#include <fstream>
int main() {
    std::ofstream out("trajectory.svg");
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='800' height='600' viewBox='0 0 800 600'>"
           "<rect width='800' height='600' fill='#f8fafc'/><path d='M120 410 L170 340 L245 285 L335 245 L430 225 L525 230 L610 260' fill='none' stroke='#4f46e5' stroke-width='9'/>"
           "<circle cx='245' cy='285' r='115' fill='none' stroke='#059669' stroke-width='4' stroke-dasharray='12 10'/></svg>";
}
