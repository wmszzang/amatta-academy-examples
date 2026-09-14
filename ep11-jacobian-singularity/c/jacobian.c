#include <math.h>
#include <stdio.h>

static const double PI = 3.14159265358979323846;

void jac(double a1, double a2, double t1, double t2, double j[2][2]) {
    double s1=sin(t1), c1=cos(t1), s12=sin(t1+t2), c12=cos(t1+t2);
    j[0][0]=-a1*s1-a2*s12; j[0][1]=-a2*s12;
    j[1][0]=a1*c1+a2*c12; j[1][1]=a2*c12;
}
double det2(const double j[2][2]) { return j[0][0]*j[1][1]-j[0][1]*j[1][0]; }
int is_singular(const double j[2][2], double eps) { return fabs(det2(j))<eps; }
int joint_vel(const double j[2][2], double vx, double vy, double q[2]) {
    double d=det2(j);
    if (is_singular(j,1e-6)) return 0;
    q[0]=(j[1][1]*vx-j[0][1]*vy)/d;
    q[1]=(-j[1][0]*vx+j[0][0]*vy)/d;
    return 1;
}
void sweep(double t2, double row[5]) {
    double j[2][2], q[2], t1=PI/6;
    double px=2*cos(t1)+1.5*cos(t1+t2), py=2*sin(t1)+1.5*sin(t1+t2);
    double r=hypot(px,py);
    jac(2,1.5,t1,t2,j);
    joint_vel(j,.1*px/r,.1*py/r,q);
    row[0]=t2; row[1]=det2(j); row[2]=q[0]; row[3]=q[1]; row[4]=hypot(q[0],q[1]);
}
int main(void) {
    double j[2][2], xd,yd,row[5], mid,lo=.001,hi=.1;
    double angles[]={.7854,.4,.2,.1,.05,.02,.01,.001}, betas[]={.7854,.2,0};
    FILE *f;
    int i;
    jac(2,1.5,.5236,.7854,j);
    xd=j[0][0]*.1+j[0][1]*.2; yd=j[1][0]*.1+j[1][1]*.2;
    f=fopen("vel.txt","wb"); if(!f) return 1; fprintf(f,"%.4f %.4f\n",xd,yd); fclose(f);
    f=fopen("jac.csv","wb"); if(!f) return 1;
    fprintf(f,"j1,j2\n%.10f,%.10f\n%.10f,%.10f\n",j[0][0],j[0][1],j[1][0],j[1][1]); fclose(f);
    f=fopen("sweep.csv","wb"); if(!f) return 1; fprintf(f,"theta2,det,qd1,qd2,qnorm\n");
    for(i=0;i<8;i++){ sweep(angles[i],row); fprintf(f,"%.10f,%.10f,%.10f,%.10f,%.10f\n",row[0],row[1],row[2],row[3],row[4]); } fclose(f);
    f=fopen("wrist.csv","wb"); if(!f) return 1; fprintf(f,"beta,abs_det\n");
    for(i=0;i<3;i++) fprintf(f,"%.10f,%.10f\n",betas[i],fabs(sin(betas[i]))); fclose(f);
    f=fopen("singular.txt","wb"); if(!f) return 1;
    fprintf(f,"outer: rank=1 r=3.5000 lost=30deg available=120deg\ninner-boundary-2link: rank=1 r=0.5000\ninternal-3link: rank=1 r=0.5000 workspace=[0,4.5]\n"); fclose(f);
    for(i=0;i<70;i++){ mid=(lo+hi)/2; sweep(mid,row); if(fabs(row[3])>PI) lo=mid; else hi=mid; }
    printf("%.4f %.4f\ndet=%.4f speed=%.4f\njoint2 limit: theta2=%.7f rad\n",xd,yd,det2(j),hypot(xd,yd),(lo+hi)/2);
    return 0;
}
