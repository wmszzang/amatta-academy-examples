#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static double wrap(double a){return atan2(sin(a),cos(a));}
bool ik2(double x,double y,double a1,double a2,int e,double& t1,double& t2){
    double c=(x*x+y*y-a1*a1-a2*a2)/(2*a1*a2);
    if(fabs(c)>1+1e-9)return 0;
    c=fmax(-1,fmin(1,c));double s=e*sqrt(fmax(0,1-c*c));
    t2=wrap(atan2(s,c));t1=wrap(atan2(y,x)-atan2(a2*s,a1+a2*c));return 1;
}
static double show(double x){return fabs(x)<.00005?0:x;}
int main(int argc,char **argv){
    double a1=2,a2=1.5,x=2.5,y=1.5,pi=acos(-1.);
    const char *unit="rad",*out="solutions.csv",*choice="both";int boundary=0;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--boundary")){boundary=1;continue;}
        if(i+1>=argc)return 2;const char *k=argv[i],*v=argv[++i];
        if(!strcmp(k,"--a1"))a1=atof(v);else if(!strcmp(k,"--a2"))a2=atof(v);
        else if(!strcmp(k,"--x"))x=atof(v);else if(!strcmp(k,"--y"))y=atof(v);
        else if(!strcmp(k,"--unit"))unit=v;else if(!strcmp(k,"--out"))out=v;
        else if(!strcmp(k,"--elbow"))choice=v;else return 2;
    }
    if(!isfinite(a1)||!isfinite(a2)||!isfinite(x)||!isfinite(y)||a1<=0||a2<=0)return 2;
    if(strcmp(unit,"rad")&&strcmp(unit,"deg"))return 2;
    if(strcmp(choice,"both")&&strcmp(choice,"down")&&strcmp(choice,"up"))return 2;
    if(boundary){x=(a1+a2)*cos(pi/3);y=(a1+a2)*sin(pi/3);}
    double q[2][2],xy[2][2];int count=0;const char *status=NULL;
    if(a1==a2&&x==0&&y==0)status="CONTINUUM";
    for(int e=1;!status&&e>=-1;e-=2){
        double t1,t2;if(!ik2(x,y,a1,a2,e,t1,t2)){status="UNREACHABLE";break;}
        double fx=a1*cos(t1)+a2*cos(t1+t2),fy=a1*sin(t1)+a2*sin(t1+t2);
        if(hypot(fx-x,fy-y)>=1e-9){status="FK_TOLERANCE_EXCEEDED";break;}
        if(count&&hypot(wrap(t1-q[0][0]),wrap(t2-q[0][1]))<1e-9)continue;
        q[count][0]=t1;q[count][1]=t2;xy[count][0]=fx;xy[count][1]=fy;count++;
    }
    // 경계에서 중복 제거된 해도 요청한 갈래로 출력한다.
    if(!status&&count==1&&!strcmp(choice,"up")){
        ik2(x,y,a1,a2,-1,q[0][0],q[0][1]);
        xy[0][0]=a1*cos(q[0][0])+a2*cos(q[0][0]+q[0][1]);xy[0][1]=a1*sin(q[0][0])+a2*sin(q[0][0]+q[0][1]);
    }
    FILE *f=fopen(out,"w");if(!f)return 3;
    const char *h="elbow,theta1,theta2,unit,fk_x,fk_y,status\n";fputs(h,f);fputs(h,stdout);
    char line[256];
    if(status){snprintf(line,sizeof(line),"none,,,%s,,,%s\n",unit,status);fputs(line,f);fputs(line,stdout);}
    else for(int i=0;i<count;i++){
        const char *name=(i==0&&!(count==1&&!strcmp(choice,"up")))?"down":"up";if(strcmp(choice,"both")&&strcmp(choice,name))continue;
        double fac=!strcmp(unit,"deg")?180/pi:1;
        snprintf(line,sizeof(line),"%s,%.4f,%.4f,%s,%.4f,%.4f,OK\n",name,show(q[i][0]*fac),show(q[i][1]*fac),unit,show(xy[i][0]),show(xy[i][1]));
        fputs(line,f);fputs(line,stdout);
    }
    fclose(f);return 0;
}
