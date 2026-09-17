#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
typedef struct { double x,y,w,k; } Point;
typedef struct { double t1,t2,w,k,err; } Result;
typedef enum { PASS, FAIL_UNREACHABLE_INNER, FAIL_UNREACHABLE_OUTER,
    FAIL_JOINT_LIMIT, FAIL_PRECISION, FAIL_LOW_MANIPULABILITY,
    FAIL_ILL_CONDITIONED } Verdict;
const char *names[]={"PASS","FAIL_UNREACHABLE_INNER","FAIL_UNREACHABLE_OUTER",
    "FAIL_JOINT_LIMIT","FAIL_PRECISION","FAIL_LOW_MANIPULABILITY","FAIL_ILL_CONDITIONED"};
double rad(double d){return d*PI/180;}
double wrap(double t){return atan2(sin(t),cos(t));}
double clean(double v){return fabs(v)<0.0000005?0:v;}
Point fk(double t1,double t2){Point p={2*cos(t1)+1.5*cos(t1+t2),2*sin(t1)+1.5*sin(t1+t2),0,0};return p;}
double manip(double t2){return 3*fabs(sin(t2));}
double cond(double t1,double t2){
    double a=-2*sin(t1)-1.5*sin(t1+t2),b=-1.5*sin(t1+t2);
    double c=2*cos(t1)+1.5*cos(t1+t2),d=1.5*cos(t1+t2);
    double E=a*a+b*b+c*c+d*d,D=fabs(a*d-b*c);
    double delta=sqrt(fmax(E*E-4*D*D,0)),lo=sqrt(fmax((E-delta)/2,0));
    return lo>1e-12?sqrt((E+delta)/2)/lo:INFINITY;
}
int ik(double x,double y,int elbow,double *t1,double *t2){
    double c=(x*x+y*y-6.25)/6;
    if(fabs(c)>1+1e-9)return 0;
    c=fmax(-1,fmin(1,c));
    *t2=atan2(elbow*sqrt(fmax(0,1-c*c)),c);
    *t1=wrap(atan2(y,x)-atan2(1.5*sin(*t2),2+1.5*c));
    *t2=wrap(*t2);return 1;
}
int allowed(double t1,double t2){
    double a=wrap(t1)*180/PI,b=wrap(t2)*180/PI;
    return a>=-30-1e-10&&a<=120+1e-10&&b>=15-1e-10&&b<=150+1e-10;
}
Verdict spec_check(double x,double y,double w_min,double k_max,double tol,Result *out){
    double r=hypot(x,y),best=-1;
    int i;
    if(r<0.5-1e-12)return FAIL_UNREACHABLE_INNER;
    if(r>3.5+1e-12)return FAIL_UNREACHABLE_OUTER;
    for(i=0;i<2;i++){
        double a,b,w;Point p;
        if(!ik(x,y,i==0?1:-1,&a,&b)||!allowed(a,b))continue;
        w=manip(b);
        if(w<=best)continue;
        best=w;p=fk(a,b);
        out->t1=a*180/PI;out->t2=b*180/PI;out->w=w;
        out->k=cond(a,b);out->err=hypot(p.x-x,p.y-y);
    }
    if(best<0)return FAIL_JOINT_LIMIT;
    if(out->err>tol)return FAIL_PRECISION;
    if(out->w<w_min)return FAIL_LOW_MANIPULABILITY;
    return out->k<=k_max?PASS:FAIL_ILL_CONDITIONED;
}
double cross(Point o,Point a,Point b){return (a.x-o.x)*(b.y-o.y)-(a.y-o.y)*(b.x-o.x);}
int cmp(const void *a,const void *b){
    const Point *p=(const Point*)a,*q=(const Point*)b;
    return p->x<q->x?-1:p->x>q->x?1:p->y<q->y?-1:p->y>q->y?1:0;
}
int hull(Point *pts,int n,Point *h){
    int i,k=0,t;qsort(pts,n,sizeof(Point),cmp);
    for(i=0;i<n;i++){while(k>=2&&cross(h[k-2],h[k-1],pts[i])<=0)k--;h[k++]=pts[i];}
    t=k+1;
    for(i=n-2;i>=0;i--){while(k>=t&&cross(h[k-2],h[k-1],pts[i])<=0)k--;h[k++]=pts[i];}
    return k-1;
}
double area(Point *p,int n){double s=0;int i;for(i=0;i<n;i++)s+=p[i].x*p[(i+1)%n].y-p[(i+1)%n].x*p[i].y;return fabs(s)/2;}
int segment_hit(Point a,Point b){
    double dx=b.x-a.x,dy=b.y-a.y,t=((1.6-a.x)*dx+(1.2-a.y)*dy)/(dx*dx+dy*dy);
    t=fmax(0,fmin(1,t));return hypot(a.x+t*dx-1.6,a.y+t*dy-1.2)<=0.35;
}
FILE *open_out(const char *dir,const char *name){char path[2048];FILE *f;
    snprintf(path,sizeof(path),"%s/%s",dir,name);f=fopen(path,"w");
    if(!f){perror(path);exit(1);}return f;
}
int main(int argc,char **argv){
    const char *dir=argc>1?argv[1]:".";
    Point pts[868],h[1736];int n=0,i,j,nh,hit=0,grid=0;
    double targets[6][2]={{2.5,1.5},{1,2.6},{3.4,.5},{.3,.2},{3.6,.2},{-1,-2}};
    FILE *f=open_out(dir,"workspace.csv");fprintf(f,"x,y,w,kappa\n");
    for(i=0;i<=30;i++)for(j=0;j<=27;j++){
        double a=rad(-30+5*i),b=rad(15+5*j);Point p=fk(a,b);p.w=manip(b);p.k=cond(a,b);pts[n++]=p;
        fprintf(f,"%.6f,%.6f,%.6f,%.6f\n",clean(p.x),clean(p.y),p.w,p.k);
    }fclose(f);
    f=open_out(dir,"spec.csv");fprintf(f,"x,y,verdict,th1,th2,w,kappa,fk_err\n");
    for(i=0;i<6;i++){
        Result r={0,0,0,0,0};Verdict v=spec_check(targets[i][0],targets[i][1],1,10,1e-6,&r);
        fprintf(f,"%.6f,%.6f,%s",targets[i][0],targets[i][1],names[v]);
        if(v==FAIL_UNREACHABLE_INNER||v==FAIL_UNREACHABLE_OUTER||v==FAIL_JOINT_LIMIT)fprintf(f,",,,,,\n");
        else fprintf(f,",%.6f,%.6f,%.6f,%.6f,%.6f\n",clean(r.t1),clean(r.t2),r.w,r.k,clean(r.err));
        printf("(%g,%g) %s w=%.4f kappa=%.4f\n",targets[i][0],targets[i][1],names[v],r.w,r.k);
    }fclose(f);
    nh=hull(pts,n,h);f=open_out(dir,"hull.csv");fprintf(f,"x,y\n");
    for(i=0;i<nh;i++)fprintf(f,"%.6f,%.6f\n",clean(h[i].x),clean(h[i].y));fclose(f);
    for(i=0;i<1400;i++)for(j=0;j<1400;j++){
        double x=-3.5+(i+.5)*.005,y=-3.5+(j+.5)*.005,a,b,r=hypot(x,y);
        if(r<.5||r>3.5)continue;
        if((ik(x,y,1,&a,&b)&&allowed(a,b))||(ik(x,y,-1,&a,&b)&&allowed(a,b)))grid++;
    }
    f=open_out(dir,"area.txt");fprintf(f,"hull_area=%.6f\ngrid_area=%.6f\nratio=%.6f\n",area(h,nh),grid*.000025,area(h,nh)/(grid*.000025));fclose(f);
    f=open_out(dir,"cspace.csv");fprintf(f,"th1,th2,state\n");
    for(i=0;i<72;i++)for(j=0;j<72;j++){
        double a=rad(-180+i*5),b=rad(-180+j*5);Point o={0,0,0,0},el={2*cos(a),2*sin(a),0,0},end=fk(a,b);
        int bad=segment_hit(o,el)||segment_hit(el,end);hit+=bad;
        fprintf(f,"%.6f,%.6f,%s\n",(double)(-180+i*5),(double)(-180+j*5),bad?"obstacle":"free");
    }fclose(f);
    printf("samples=%d hull_vertices=%d hull_area=%.7f grid_area=%.7f\n",n,nh,area(h,nh),grid*.000025);
    printf("C-obstacle=%d C-free=%d\n",hit,5184-hit);return 0;
}
