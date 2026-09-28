#include <math.h>
#include "quat.h"
Quat qmul(Quat a,Quat b) {
    Quat r={a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
    return r;
}
Quat qnorm(Quat a) {
    double n=sqrt(a.w*a.w+a.x*a.x+a.y*a.y+a.z*a.z);
    Quat r={a.w/n,a.x/n,a.y/n,a.z/n}; return r;
}
Quat conjq(Quat a) { Quat r={a.w,-a.x,-a.y,-a.z}; return r; }
Quat axis_deg(double x,double y,double z,double degrees) {
    double h=degrees*acos(-1.0)/360, n=sqrt(x*x+y*y+z*z);
    Quat r={cos(h),x/n*sin(h),y/n*sin(h),z/n*sin(h)}; return r;
}
