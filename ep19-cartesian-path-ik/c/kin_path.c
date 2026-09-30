/* EP.19: 모든 내부 각도는 라디안이며 CSV에서만 도로 변환한다. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct Pt { double x, y; } Pt;
typedef struct Joint { double a, b; int ok; } Joint;
static const double pi=3.14159265358979323846, l1=1.2, l2=.9;
static const Pt p0={1.6,.3}, p1={.6,1.5};
static double deg(double v) { return v*180.0/pi; }
static double clean(double v) { return fabs(v)<.00005?0:v; }
static Pt point(double x,double y) { Pt p={x,y}; return p; }
static Pt mix(Pt a,Pt b,double u) { return point(a.x+u*(b.x-a.x),a.y+u*(b.y-a.y)); }
static Pt fk2(Joint q) { return point(l1*cos(q.a)+l2*cos(q.a+q.b),l1*sin(q.a)+l2*sin(q.a+q.b)); }
static Joint ik2(Pt p) {
    Joint q={0,0,0};
    double rho=hypot(p.x,p.y), c;
    if (rho<fabs(l1-l2)-1e-12 || rho>l1+l2+1e-12) return q;
    c=(p.x*p.x+p.y*p.y-l1*l1-l2*l2)/(2*l1*l2);
    if(c>1)c=1;
    if(c< -1)c= -1;
    q.b=acos(c); q.a=atan2(p.y,p.x)-atan2(l2*sin(q.b),l1+l2*cos(q.b)); q.ok=1;
    return q;
}
static Joint joint_mix(Joint a,Joint b,double u) { Joint q={a.a+u*(b.a-a.a),a.b+u*(b.b-a.b),1};return q; }
static double dist(Pt a,Pt b) { return hypot(a.x-b.x,a.y-b.y); }
static double dev(Pt p) { return fabs((p1.x-p0.x)*(p.y-p0.y)-(p1.y-p0.y)*(p.x-p0.x))/dist(p0,p1); }
static double trap_s(double t,double D,double T,double ta) {
    double v=D/(T-ta),a=v/ta;
    if(t<ta)return .5*a*t*t;
    if(t<T-ta)return .5*a*ta*ta+v*(t-ta);
    if(t<T)return D-.5*a*(T-t)*(T-t);
    return D;
}
static FILE *file(const char *name,const char *header) {
    FILE *f=fopen(name,"w");
    if(!f){perror(name);exit(2);}
    fprintf(f,"%s\n",header);return f;
}
static void row(FILE *f,const double *values,int count) {
    int i;for(i=0;i<count;i++)fprintf(f,"%s%.4f",i?",":"",clean(values[i]));fputc('\n',f);
}
static void compare(void) {
    Joint qa=ik2(p0),qb=ik2(p1);int i,failed=0;
    double maxdev=0,length=0;Pt prev=fk2(qa);
    FILE *f=file("compare.csv","u,th1_joint,th2_joint,x_joint,y_joint,dev,x_cart,y_cart,th1_cart,th2_cart");
    for(i=0;i<=10;i++) {
        double u=i/10.; Joint q=joint_mix(qa,qb,u);Pt p=fk2(q),pc=mix(p0,p1,u);Joint qc=ik2(pc);
        double v[]={u,deg(q.a),deg(q.b),p.x,p.y,dev(p),pc.x,pc.y,deg(qc.a),deg(qc.b)};
        row(f,v,10);
    }
    fclose(f);
    for(i=0;i<=10000;i++) {
        Pt p=fk2(joint_mix(qa,qb,i/10000.));double d=dev(p);
        if(d>maxdev)maxdev=d;
        if(i)length+=dist(prev,p);
        prev=p;
    }
    printf("max_deviation=%.4f\njoint_length=%.4f\nline_length=%.4f\n",maxdev,length,dist(p0,p1));
    f=file("unreachable.csv","x,y,th1_rad,th2_rad,status");
    for(i=0;i<=10;i++) {
        Pt p=mix(point(.35,0),point(-.35,0),i/10.);Joint q=ik2(p);
        if(!q.ok){failed++;fprintf(f,"%.4f,%.4f,,,UNREACHABLE\n",clean(p.x),clean(p.y));}
        else fprintf(f,"%.4f,%.4f,%.4f,%.4f,OK\n",clean(p.x),clean(p.y),q.a,q.b);
    }
    fclose(f);
    /* 진단은 모든 점을 기록하되 하나라도 실패하면 명령을 생성하지 않는다. */
    if(failed!=9){fprintf(stderr,"unexpected reachability\n");exit(3);}
    printf("unreachable=%d/11; commands=BLOCKED\n",failed);
}
static void arcs(void) {
    double a0=pi/6,a1=2*pi/3,err=0;int i;Pt first,last;
    Pt c=point(l1*cos(a0),l1*sin(a0));
    FILE *fa=file("arc.csv","x,y"),*fs=file("sweep.csv","x,y");
    for(i=0;i<=6;i++) {
        double a=a0+(a1-a0)*i/6.;Joint q={a0,pi*i/12.,1};
        Pt p=point(c.x+l2*cos(a),c.y+l2*sin(a)),s=fk2(q);
        double va[]={p.x,p.y},vs[]={s.x,s.y},d=dist(p,s);
        row(fa,va,2);row(fs,vs,2);if(d>err)err=d;if(!i)first=p;last=p;
    }
    fclose(fa);fclose(fs);
    if(err>1e-12){fprintf(stderr,"arc mismatch\n");exit(3);}
    printf("arc_length=%.4f; chord=%.4f; match_tol=1e-12\n",l2*(a1-a0),dist(first,last));
}
static void trapezoid(void) {
    double L=dist(p0,p1),m1=0,m2=0,wmin=100,wmax=0,pt=0;Joint prev=ik2(p0);int i;
    FILE *f=file("path.csv","t,s,x,y,th1,th2,dth1,dth2,w");
    for(i=0;i<=20;i++) {
        double t=i*.1,s=trap_s(t,L,2.,.5),w,d1=0,d2=0;Pt p=mix(p0,p1,s/L);Joint q=ik2(p);
        if(!q.ok){fclose(f);fprintf(stderr,"UNREACHABLE: no commands\n");exit(3);}
        w=fabs(l1*l2*sin(q.b));
        if(i){d1=deg(q.a-prev.a)/(t-pt);d2=deg(q.b-prev.b)/(t-pt);}
        if(fabs(d1)>m1)m1=fabs(d1);if(fabs(d2)>m2)m2=fabs(d2);
        if(w<wmin)wmin=w;if(w>wmax)wmax=w;
        if(!i)fprintf(f,"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,,,%.4f\n",t,s,p.x,p.y,deg(q.a),deg(q.b),w);
        else {double v[]={t,s,p.x,p.y,deg(q.a),deg(q.b),d1,d2,w};row(f,v,9);}
        prev=q;pt=t;
    }
    fclose(f);printf("sample_speed_max=%.3f,%.3f\nw_range=%.4f,%.4f\n",m1,m2,wmin,wmax);
}
static void polynomials(void) {
    Joint qa=ik2(p0),qb=ik2(p1);double d1=qb.a-qa.a,d2=qb.b-qa.b,mid=0;int i;
    FILE *fc=file("cubic.csv","t,th1,dth1,ddth1,th2,dth2,ddth2"),*fd=file("dev.csv","t,x,y,dev");
    for(i=0;i<=20;i++) {
        double u=i/20.,h=3*u*u-2*u*u*u,dh=(6*u-6*u*u)/2,ddh=(6-12*u)/4,t=i/10.;
        Joint q=joint_mix(qa,qb,h);Pt p=fk2(q);
        double vc[]={t,deg(q.a),deg(d1*dh),deg(d1*ddh),deg(q.b),deg(d2*dh),deg(d2*ddh)};
        double vd[]={t,p.x,p.y,dev(p)};
        row(fc,vc,7);row(fd,vd,4);if(i==10)mid=dev(p);
    }
    fclose(fc);fclose(fd);
    if(fabs(mid-.2004845)>1e-6)exit(3);
    printf("cubic_mid_deviation=%.4f\ncubic_peak_speed=%.3f\nquintic_peak_speed=%.3f\n",mid,deg(d1)*1.5/2,deg(d1)*1.875/2);
}
static void graph(void) {
    int i;Joint qa=ik2(p0),qb=ik2(p1);FILE *f=fopen("trajectory.svg","w");if(!f)exit(2);
    fprintf(f,"<svg xmlns='http://www.w3.org/2000/svg' width='800' height='640' viewBox='0 0 800 640'><rect width='800' height='640' fill='#f4f6fb'/><text x='50' y='40' font-size='24'>EP.19: joint / Cartesian path (m)</text><path d='M 80 70 V 570 H 730' fill='none' stroke='#253047'/><polyline fill='none' stroke='#4f46e5' stroke-width='4' points='");
    for(i=0;i<=100;i++){Pt p=fk2(joint_mix(qa,qb,i/100.));fprintf(f,"%.3f,%.3f ",80+p.x*240,570-p.y*240);}
    fprintf(f,"'/><path d='M %.3f %.3f L %.3f %.3f' stroke='#059669' stroke-width='4'/><text x='90' y='610' font-size='20'>Same scale on x and y; joint path is curved.</text></svg>",80+p0.x*240,570-p0.y*240,80+p1.x*240,570-p1.y*240);fclose(f);
}
int main(void) { compare();arcs();trapezoid();polynomials();graph();return 0; }
