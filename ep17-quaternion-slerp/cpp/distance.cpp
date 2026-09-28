#include <math.h>
#include "quat.hpp"
double geodesic_deg(Quat a,Quat b) {
    double d=fabs(a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z);
    return 360/acos(-1.0)*acos(fmin(1.0,d));
}
