/* EP.22: 원값으로 계산하고 출력에서만 반올림한다. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
typedef struct { double x,y; } Pt;
typedef struct { double x,y,theta; } Pose;
typedef struct { double a,b; int reachable; } Joint;
static Pt fk(Joint q,double l1,double l2) { Pt p={l1*cos(q.a)+l2*cos(q.a+q.b),l1*sin(q.a)+l2*sin(q.a+q.b)};return p; }
static Joint ik(Pt p,double l1,double l2) {
 double c=(p.x*p.x+p.y*p.y-l1*l1-l2*l2)/(2*l1*l2);Joint q={0,0,0};
 if(fabs(c)>1+1e-12)return q;
 c=fmax(-1,fmin(1,c));q.b=atan2(sqrt(fmax(0,1-c*c)),c);
 q.a=atan2(p.y,p.x)-atan2(l2*sin(q.b),l1+l2*cos(q.b));q.reachable=1;return q;
}
static void diff_ik(double v,double w,double r,double track,double out[4]) {
 out[0]=v-w*track/2;out[1]=v+w*track/2;out[2]=out[0]/r;out[3]=out[1]/r;
}
static void mecanum(double vx,double vy,double w,double r,double lx,double ly,double out[4]) {
 double k=lx+ly;out[0]=(vx-vy-k*w)/r;out[1]=(vx+vy+k*w)/r;
 out[2]=(vx+vy-k*w)/r;out[3]=(vx-vy+k*w)/r;
}
static Pose bicycle(Pose p,double v,double delta,double wheelbase,double dt) {
 Pose next={p.x+v*cos(p.theta)*dt,p.y+v*sin(p.theta)*dt,p.theta+v*tan(delta)*dt/wheelbase};return next;
}
static Pt transform(Pt p,double a,Pt t) {Pt q={cos(a)*p.x-sin(a)*p.y+t.x,sin(a)*p.x+cos(a)*p.y+t.y};return q;}
static void rigid_svd(Pt *q,Pt *p,int n,double *angle,Pt *translation,int *corrected) {
 Pt cq={0,0},cp={0,0};double h[2][2]={{0,0},{0,0}};
 for(int i=0;i<n;i++){cq.x+=q[i].x/n;cq.y+=q[i].y/n;cp.x+=p[i].x/n;cp.y+=p[i].y/n;}
 for(int i=0;i<n;i++){double x=q[i].x-cq.x,y=q[i].y-cq.y,X=p[i].x-cp.x,Y=p[i].y-cp.y;h[0][0]+=x*X;h[0][1]+=x*Y;h[1][0]+=y*X;h[1][1]+=y*Y;}
 /* H의 SVD: H^T H의 고유벡터 V와 U=HV/S를 구한다. */
 double aa=h[0][0]*h[0][0]+h[1][0]*h[1][0],bb=h[0][0]*h[0][1]+h[1][0]*h[1][1],dd=h[0][1]*h[0][1]+h[1][1]*h[1][1];
 double phi=.5*atan2(2*bb,aa-dd),c=cos(phi),s=sin(phi);
 double v[2][2]={{c,-s},{s,c}},u[2][2],sigma[2];
 for(int j=0;j<2;j++){u[0][j]=h[0][0]*v[0][j]+h[0][1]*v[1][j];u[1][j]=h[1][0]*v[0][j]+h[1][1]*v[1][j];sigma[j]=hypot(u[0][j],u[1][j]);
  if(sigma[j]<1e-12){fprintf(stderr,"Degenerate correspondence\n");exit(2);}u[0][j]/=sigma[j];u[1][j]/=sigma[j];}
 double du=u[0][0]*u[1][1]-u[0][1]*u[1][0],dv=v[0][0]*v[1][1]-v[0][1]*v[1][0];
 *corrected=du*dv<0;if(*corrected){v[0][1]*=-1;v[1][1]*=-1;}
 double r00=v[0][0]*u[0][0]+v[0][1]*u[0][1],r10=v[1][0]*u[0][0]+v[1][1]*u[0][1];
 *angle=atan2(r10,r00);Pt origin={0,0},rc=transform(cq,*angle,origin);translation->x=cp.x-rc.x;translation->y=cp.y-rc.y;
}
static void rigid_closed(Pt *q,Pt *p,int n,double *angle,Pt *t) {
 Pt cq={0,0},cp={0,0};for(int i=0;i<n;i++){cq.x+=q[i].x/n;cq.y+=q[i].y/n;cp.x+=p[i].x/n;cp.y+=p[i].y/n;}
 double num=0,den=0;for(int i=0;i<n;i++){double x=q[i].x-cq.x,y=q[i].y-cq.y,X=p[i].x-cp.x,Y=p[i].y-cp.y;num+=x*Y-y*X;den+=x*X+y*Y;}
 *angle=atan2(num,den);Pt zero={0,0},rc=transform(cq,*angle,zero);t->x=cp.x-rc.x;t->y=cp.y-rc.y;
}
static int project(double x,double y,double z,Pt *uv) {if(z<=0)return 0;uv->x=320+600*x/z;uv->y=240+600*y/z;return 1;}
static int backproject(double ul,double vl,double ur,Pose *p) {double d=ul-ur;if(d<=0)return 0;p->theta=600*.12/d;p->x=(ul-320)*p->theta/600;p->y=(vl-240)*p->theta/600;return 1;}
static void rotation(double roll,double pitch,double yaw,double r[3][3]) {
 double cr=cos(roll),sr=sin(roll),cp=cos(pitch),sp=sin(pitch),cy=cos(yaw),sy=sin(yaw);
 r[0][0]=cy*cp;r[0][1]=cy*sp*sr-sy*cr;r[0][2]=cy*sp*cr+sy*sr;
 r[1][0]=sy*cp;r[1][1]=sy*sp*sr+cy*cr;r[1][2]=sy*sp*cr-cy*sr;
 r[2][0]=-sp;r[2][1]=cp*sr;r[2][2]=cp*cr;
}
static void euler(double r[3][3],double a[3]) {
 double cp=hypot(r[0][0],r[1][0]);a[1]=atan2(-r[2][0],cp);
 if(cp<1e-10){a[2]=0;a[0]=atan2(-r[1][2],r[1][1]);}
 else {a[2]=atan2(r[1][0],r[0][0]);a[0]=atan2(r[2][1],r[2][2]);}
}
static void check(int ok,const char *name) {if(!ok){fprintf(stderr,"FAIL: %s\n",name);exit(3);}}
static FILE *open_out(const char *dir,const char *name) {char path[2048];snprintf(path,sizeof(path),"%s/%s",dir,name);FILE *f=fopen(path,"w");if(!f){perror(path);exit(2);}return f;}
int main(int argc,char **argv) {
 const char *out=argc>1?argv[1]:"out";FILE *f;double diff[4],mec[4];
 diff_ik(.6,-.5,.04,.36,diff);mecanum(.9,-.4,.25,.05,.30,.26,mec);
 check(fabs(diff[0]-.69)<1e-12&&fabs(mec[0]-23.2)<1e-12&&fabs(mec[3]-28.8)<1e-12,"wheel order");
 f=open_out(out,"wheels.csv");fprintf(f,"problem,wheel,linear_m_s,angular_rad_s\n261,L,%.4f,%.4f\n261,R,%.4f,%.4f\n",diff[0],diff[2],diff[1],diff[3]);
 const char *names[]={"FL","FR","RL","RR"};for(int i=0;i<4;i++)fprintf(f,"483,%s,%.4f,%.4f\n",names[i],mec[i]*.05,mec[i]);fclose(f);
 Pose od={0,0,0};double dl=120*2*PI*.04/1024,dr=150*2*PI*.04/1024,dc=(dl+dr)/2,dth=(dr-dl)/.36;
 f=open_out(out,"odom.csv");fprintf(f,"step,x,y,theta\n");for(int i=0;i<=40;i++){fprintf(f,"%d,%.4f,%.4f,%.4f\n",i,od.x,od.y,od.theta);if(i<40){od.x+=dc*cos(od.theta+dth/2);od.y+=dc*sin(od.theta+dth/2);od.theta+=dth;}}fclose(f);
 Pose steer={0,0,0};double delta=atan(.225),ain=atan(2.7/11.2),aout=atan(2.7/12.8);
 f=open_out(out,"steer.csv");fprintf(f,"t,x,y,theta,delta_in,delta_out\n");for(int i=0;i<=20;i++){fprintf(f,"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",i*.1,steer.x,steer.y,steer.theta,ain,aout);if(i<20)steer=bicycle(steer,1.5,delta,2.7,.1);}fclose(f);
 Pt pts[7];Joint sol[7];f=open_out(out,"arc_path.csv");fprintf(f,"i,angle,x,y,th1,th2,w,reachable\n");
 for(int i=0;i<7;i++){double a=(30+20*i)*PI/180;pts[i].x=1.2+.5*cos(a);pts[i].y=.6+.5*sin(a);sol[i]=ik(pts[i],1.2,.9);fprintf(f,"%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,1\n",i,30+20*i,pts[i].x,pts[i].y,sol[i].a*180/PI,sol[i].b*180/PI,fabs(1.2*.9*sin(sol[i].b)));}fclose(f);
 f=open_out(out,"jointlerp_gap.csv");fprintf(f,"i,x_lerp,y_lerp,x_arc,y_arc,gap\n");double maxgap=0;
 for(int i=0;i<7;i++){Joint q={sol[0].a+(sol[6].a-sol[0].a)*i/6,sol[0].b+(sol[6].b-sol[0].b)*i/6,1};Pt p=fk(q,1.2,.9);double gap=hypot(p.x-pts[i].x,p.y-pts[i].y);maxgap=fmax(maxgap,gap);fprintf(f,"%d,%.4f,%.4f,%.4f,%.4f,%.4f\n",i,p.x,p.y,pts[i].x,pts[i].y,gap);}fclose(f);
 Pt Q[4]={{0,0},{1.2,0},{1.2,.8},{0,.8}},P[4]={{1.5,-.8},{2.4829824531,-.1117082764},{2.0241213041,.5436133591},{1.0411388509,-.1446783646}},t,tc;double angle,ac;int corrected;
 rigid_svd(Q,P,4,&angle,&t,&corrected);rigid_closed(Q,P,4,&ac,&tc);double err=0;
 for(int i=0;i<4;i++){Pt p=transform(Q[i],angle,t);err=fmax(err,hypot(p.x-P[i].x,p.y-P[i].y));}
 check(fabs(angle-ac)<1e-10&&err<1e-8,"SVD and closed form");
 f=open_out(out,"calib.txt");fprintf(f,"SVD %.4f %.4f %.4f %.4f\nCLOSED %.4f %.4f %.4f\n",angle*180/PI,t.x,t.y,err,ac*180/PI,tc.x,tc.y);fclose(f);
 Pt uv;Pose xyz;check(project(.36,-.15,3,&uv),"project");check(backproject(392,210,368,&xyz),"backproject");
 f=open_out(out,"camera.csv");fprintf(f,"u,v,X,Y,Z\n%.4f,%.4f,%.4f,%.4f,%.4f\n",uv.x,uv.y,xyz.x,xyz.y,xyz.theta);fclose(f);
 double a1=.4,a2=.9;double A[3][3]={{cos(a1),-sin(a1),2*cos(a1)},{sin(a1),cos(a1),2*sin(a1)},{0,0,1}},B[3][3]={{cos(a2),-sin(a2),1.5*cos(a2)},{sin(a2),cos(a2),1.5*sin(a2)},{0,0,1}},T[3][3]={{0}};for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)T[i][j]+=A[i][k]*B[k][j];Pt p={T[0][2],T[1][2]},goal={1.2,.7};Joint down=ik(goal,1,1);double r[3][3],a[3];rotation(-.4,.7,1.1,r);euler(r,a);
 double vx=(-2*sin(.3)-1.5*sin(.9))*.5+(-1.5*sin(.9))*(-.2),vy=(2*cos(.3)+1.5*cos(.9))*.5+1.5*cos(.9)*(-.2);
 f=open_out(out,"rejudge.csv");fprintf(f,"id,a,b,c\n237,%.4f,%.4f,0.0000\n222,%.4f,%.4f,0.0000\n239,%.4f,%.4f,%.4f\n252,%.4f,%.4f,%.4f\n",p.x,p.y,down.a*180/PI,down.b*180/PI,vx,vy,3*sin(.6),a[0],a[1],a[2]);fclose(f);
 Pt far={3,0};check(!ik(far,1,1).reachable,"UNREACHABLE");check(!project(1,2,0,&uv)&&!project(1,2,-1,&uv),"invalid depth");check(!backproject(10,10,10,&xyz),"zero disparity");
 for(int pole=-1;pole<=1;pole+=2){double rebuilt[3][3];rotation(-.4,pole*PI/2,1.1,r);euler(r,a);rotation(a[0],a[1],a[2],rebuilt);for(int i=0;i<3;i++)for(int j=0;j<3;j++)check(fabs(r[i][j]-rebuilt[i][j])<1e-10,"gimbal lock");}
 Pt mirrored[4];for(int i=0;i<4;i++){mirrored[i].x=-Q[i].x;mirrored[i].y=Q[i].y;}rigid_svd(Q,mirrored,4,&ac,&tc,&corrected);check(corrected,"reflection correction");
 printf("diff %.4f %.4f %.4f %.4f\nmecanum %.4f %.4f %.4f %.4f\nodom %.4f %.4f %.4f %.4f deg\nbicycle %.4f %.4f %.4f\nSVD %.4f %.4f %.4f\nmax sampled gap %.4f\nPASS: units, wheel order, unreachable, invalid camera, both gimbal poles, reflection\n",diff[0],diff[1],diff[2],diff[3],mec[0],mec[1],mec[2],mec[3],od.x,od.y,od.theta,od.theta*180/PI,steer.x,steer.y,steer.theta,angle*180/PI,t.x,t.y,maxgap);
 return 0;
}
