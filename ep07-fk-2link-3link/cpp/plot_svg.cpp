#include <fstream>
int main() {
    std::ofstream out("arm.svg");
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='600' height='400'><line x1='80' y1='320' x2='390' y2='170' stroke='#4f46e5' stroke-width='12'/><line x1='390' y1='170' x2='450' y2='80' stroke='#059669' stroke-width='12'/></svg>";
}
