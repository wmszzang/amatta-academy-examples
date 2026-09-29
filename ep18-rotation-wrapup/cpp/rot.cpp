// C++11에서도 같은 수치 규약을 사용하는 완결 구현.
/* 열벡터·능동 회전, 라디안 입력, 쿼터니언 wxyz 순서. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

typedef struct { double v[3][3]; } Mat3;
typedef struct { double w,x,y,z; } Q;
static const double PI=3.14159265358979323846;
static double clamp(double x) { return fmax(-1.,fmin(1.,x)); }
static Q unit(Q q) {
    double n=sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    assert(n>0); q.w/=n;q.x/=n;q.y/=n;q.z/=n;return q;
}
static Mat3 mul(Mat3 a, Mat3 b) {
    Mat3 r={0};int i,j,k;
    for(i=0;i<3;i++)for(j=0;j<3;j++)for(k=0;k<3;k++)r.v[i][j]+=a.v[i][k]*b.v[k][j];
    return r;
}
static Mat3 euler_zyx_to_R(double r,double p,double y) {
    double sr=sin(r),cr=cos(r),sp=sin(p),cp=cos(p),sy=sin(y),cy=cos(y);
    Mat3 a={{{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},
             {sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}}};return a;
}
static int euler_zyx(Mat3 R,double *r,double *p,double *y) {
    double sy=hypot(R.v[0][0],R.v[1][0]);*p=atan2(-R.v[2][0],sy);
    if(sy<1e-6){*r=atan2(-R.v[1][2],R.v[1][1]);*y=0;return 1;}
    *r=atan2(R.v[2][1],R.v[2][2]);*y=atan2(R.v[1][0],R.v[0][0]);return 0;
}
static Mat3 rodrigues(double x,double y,double z,double t) {
    double n=sqrt(x*x+y*y+z*z);Mat3 K,R,K2;int i,j;assert(n>0);
    x/=n;y/=n;z/=n;
    { Mat3 tmp={{{0,-z,y},{z,0,-x},{-y,x,0}}}; K=tmp; }
    K2=mul(K,K);
    for(i=0;i<3;i++)for(j=0;j<3;j++)R.v[i][j]=(i==j)+sin(t)*K.v[i][j]+(1-cos(t))*K2.v[i][j];
    return R;
}
static Mat3 q_to_R(Q q) {
    double w,x,y,z;q=unit(q);w=q.w;x=q.x;y=q.y;z=q.z;
    { Mat3 R={{{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)},
              {2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)},
              {2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}}};return R; }
}
static Q R_to_q(Mat3 R) {
    double q[4]={0},tr=R.v[0][0]+R.v[1][1]+R.v[2][2],s;int i,j,k;
    if(tr>0){s=2*sqrt(1+tr);q[0]=s/4;q[1]=(R.v[2][1]-R.v[1][2])/s;
        q[2]=(R.v[0][2]-R.v[2][0])/s;q[3]=(R.v[1][0]-R.v[0][1])/s;}
    else {i=0;if(R.v[1][1]>R.v[i][i])i=1;if(R.v[2][2]>R.v[i][i])i=2;
        j=(i+1)%3;k=(i+2)%3;s=2*sqrt(fmax(0,1+R.v[i][i]-R.v[j][j]-R.v[k][k]));
        q[0]=(R.v[k][j]-R.v[j][k])/s;q[i+1]=s/4;
        q[j+1]=(R.v[j][i]+R.v[i][j])/s;q[k+1]=(R.v[k][i]+R.v[i][k])/s;}
    {Q r={q[0],q[1],q[2],q[3]};return unit(r);}
}
static Q hamilton(Q a,Q b) {
    Q q={a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
         a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
         a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
         a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};return q;
}
static double dot(Q a,Q b){return a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;}
static Q slerp(Q a,Q b,double t) {
    double d,f,g,theta;Q q;a=unit(a);b=unit(b);d=dot(a,b);
    if(d<0){b.w=-b.w;b.x=-b.x;b.y=-b.y;b.z=-b.z;d=-d;}d=clamp(d);
    if(d>.9995){f=1-t;g=t;}else{theta=acos(d);f=sin((1-t)*theta)/sin(theta);g=sin(t*theta)/sin(theta);}
    q.w=f*a.w+g*b.w;q.x=f*a.x+g*b.x;q.y=f*a.y+g*b.y;q.z=f*a.z+g*b.z;
    return d>.9995?unit(q):q;
}
static double geodesic_deg(Q a,Q b){return 2*acos(clamp(fabs(dot(unit(a),unit(b)))))*180/PI;}
static double gyro_heading(const double *w,int n,double dt,double b,double th0){
    int i;assert(n>0&&dt>0);for(i=1;i<n;i++)th0+=((w[i-1]-b)+(w[i]-b))*dt/2;return th0;
}
static double det(Mat3 R){int i;double d=0;for(i=0;i<3;i++)d+=R.v[0][i]*(R.v[1][(i+1)%3]*R.v[2][(i+2)%3]-R.v[1][(i+2)%3]*R.v[2][(i+1)%3]);return d;}
static double err(Mat3 a,Mat3 b){int i,j;double e=0;for(i=0;i<3;i++)for(j=0;j<3;j++)e=fmax(e,fabs(a.v[i][j]-b.v[i][j]));return e;}
static double clean(double x){return fabs(x)<.00005?0:x;}
static void number(FILE *f,double x){fprintf(f,",%.4f",clean(x));}
static void matrix(FILE *f,Mat3 R){int i,j;for(i=0;i<3;i++)for(j=0;j<3;j++)number(f,R.v[i][j]);}
static void quat(FILE *f,Q q){number(f,q.w);number(f,q.x);number(f,q.y);number(f,q.z);}
static FILE *open_csv(const char *name,const char *header){FILE *f=fopen(name,"wb");if(!f){perror(name);exit(1);}fprintf(f,"%s\n",header);return f;}
static void close_csv(FILE *f,const char *name){fclose(f);printf("%s: verification max_error < 1e-10: PASS\n",name);}

int main(void) {
    FILE *f;int i,j;double r,p,y;Mat3 R,B;Q q;
    Q id={1,0,0,0},z90={1,0,0,1},x90={1,1,0,0};z90=unit(z90);x90=unit(x90);
    f=open_csv("euler.csv","case,r00,r01,r02,r10,r11,r12,r20,r21,r22,roll,pitch,yaw,lock");
    {const char *names[]={"general","lock_a","lock_b","lock_c","lock_negative"};
     double a[5][3]={{.3,-.5,1.2},{.4,PI/2,.9},{0,PI/2,.5},{1,PI/2,1.5},{.4,-PI/2,.9}};
     for(i=0;i<5;i++){int lock;R=euler_zyx_to_R(a[i][0],a[i][1],a[i][2]);lock=euler_zyx(R,&r,&p,&y);
         assert(err(R,euler_zyx_to_R(r,p,y))<1e-10);fprintf(f,"%s",names[i]);matrix(f,R);number(f,r);number(f,p);number(f,y);fprintf(f,",%d\n",lock);}}
    close_csv(f,"euler.csv");
    f=open_csv("rod.csv","kx,ky,kz,theta,r00,r01,r02,r10,r11,r12,r20,r21,r22,det");
    {double a[3][4]={{1,2,2,1},{0,0,1,PI/2},{1,1,1,2*PI/3}};
     for(i=0;i<3;i++){R=rodrigues(a[i][0],a[i][1],a[i][2],a[i][3]);assert(err(R,q_to_R(R_to_q(R)))<1e-10);
         fprintf(f,"%.4f",a[i][0]);for(j=1;j<4;j++)number(f,a[i][j]);matrix(f,R);number(f,det(R));fprintf(f,"\n");}}
    close_csv(f,"rod.csv");
    f=open_csv("quat.csv","w,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22,orth_err");
    {Q a[2]={{.7071,0,.7071,0},{1,1,1,1}};Mat3 I={{{1,0,0},{0,1,0},{0,0,1}}};
     for(i=0;i<2;i++){int k;double e;R=q_to_R(a[i]);for(j=0;j<3;j++)for(k=0;k<3;k++)B.v[j][k]=R.v[k][j];
         e=err(mul(B,R),I);assert(e<1e-10);assert(err(R,q_to_R(R_to_q(R)))<1e-10);
         fprintf(f,"%.4f",a[i].w);number(f,a[i].x);number(f,a[i].y);number(f,a[i].z);matrix(f,R);number(f,e);fprintf(f,"\n");}}
    close_csv(f,"quat.csv");
    f=open_csv("hamilton.csv","order,w,x,y,z,geo_deg");
    {const char *names[]={"290_unit_z_z","490_raw_identity_x180","490_raw_z_x","490_raw_x_z"};
     Q x180={0,1,0,0};Q a[4]={z90,id,z90,x90},b[4]={z90,x180,x90,z90};
     for(i=0;i<4;i++){q=hamilton(a[i],b[i]);if(i==0)q=unit(q);assert(err(q_to_R(q),mul(q_to_R(a[i]),q_to_R(b[i])))<1e-10);
         fprintf(f,"%s",names[i]);quat(f,q);number(f,geodesic_deg(id,q));fprintf(f,"\n");}}
    close_csv(f,"hamilton.csv");
    f=open_csv("slerp.csv","t,w,x,y,z,angle_deg,mode");
    {double t[]={0,.1,.25,.5,.75,.9,1};for(i=0;i<7;i++){q=slerp(id,z90,t[i]);assert(err(q_to_R(q),q_to_R(R_to_q(q_to_R(q))))<1e-10);
         fprintf(f,"%.4f",t[i]);quat(f,q);number(f,geodesic_deg(id,q));fprintf(f,",SLERP\n");}
     {Q nearq={1,0,0,.01};q=slerp(id,nearq,.5);fprintf(f,"0.5000");quat(f,q);number(f,geodesic_deg(id,q));fprintf(f,",LERP_NEAR\n");}}
    close_csv(f,"slerp.csv");
    f=open_csv("dist.csv","case,deg");
    {const char *names[]={"right_angle","same","opposite_sign","half_turn"};Q neg={-1,0,0,0},half={0,0,0,1};Q a[4]={z90,id,neg,half};
     for(i=0;i<4;i++){assert(err(q_to_R(a[i]),q_to_R(R_to_q(q_to_R(a[i]))))<1e-10);fprintf(f,"%s",names[i]);number(f,geodesic_deg(id,a[i]));fprintf(f,"\n");}}
    close_csv(f,"dist.csv");
    f=open_csv("gyro.csv","i,omega,corrected,theta");
    {double w[]={.1,.2,.2,.1};for(i=0;i<4;i++){fprintf(f,"%d",i);number(f,w[i]);number(f,w[i]-.1);number(f,gyro_heading(w,i+1,.5,.1,0));fprintf(f,"\n");}
     r=gyro_heading(w,4,.5,.1,0);q.w=cos(r/2);q.x=q.y=0;q.z=sin(r/2);assert(fabs(geodesic_deg(id,q)-r*180/PI)<1e-10);}
    close_csv(f,"gyro.csv");
    return 0;
}
