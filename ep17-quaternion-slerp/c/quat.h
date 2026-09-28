#ifndef QUAT_H
#define QUAT_H
typedef struct { double w,x,y,z; } Quat;
Quat qmul(Quat a, Quat b);
Quat qnorm(Quat a);
Quat conjq(Quat a);
Quat axis_deg(double x,double y,double z,double degrees);
Quat slerp(Quat a,Quat b,double t);
Quat nlerp(Quat a,Quat b,double t);
double geodesic_deg(Quat a,Quat b);
#endif
