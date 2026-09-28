#include <math.h>
#include "quat.hpp"
Quat nlerp(Quat a,Quat b,double t) {
    Quat q={a.w+t*(b.w-a.w),a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};
    return qnorm(q);
}
Quat slerp(Quat a,Quat b,double t) {
    double d=a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;
    if(d<0) { b.w=-b.w;b.x=-b.x;b.y=-b.y;b.z=-b.z;d=-d; }
    if(d>.9995) return nlerp(a,b,t);
    double th=acos(fmax(-1,fmin(1,d))),s=sin(th);
    double u=sin((1-t)*th)/s,v=sin(t*th)/s;
    Quat q={u*a.w+v*b.w,u*a.x+v*b.x,u*a.y+v*b.y,u*a.z+v*b.z};return q;
}
