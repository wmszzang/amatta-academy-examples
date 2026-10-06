/* EP.24: 접촉을 포함한 충돌 검사와 고정 표본 재생. 실제 장비 제어용이 아니다. */
#include <stdio.h>
#include <math.h>
#include <assert.h>

typedef struct {double x,y;} Point;
typedef struct {int r,c;} Cell;
typedef struct {Point s,a,b; double d; const char *status;} Event;
typedef struct {Point nodes[256],path[256]; int parent[256],count,length,events; Event log[256];} Result;

double distance(Point a,Point b){return hypot(a.x-b.x,a.y-b.y);}
double segment_distance(Point a,Point b,Point center){
    double dx=b.x-a.x,dy=b.y-a.y,n=dx*dx+dy*dy;
    double t=n==0?0:((center.x-a.x)*dx+(center.y-a.y)*dy)/n;
    Point p; if(t<0)t=0; if(t>1)t=1;
    p.x=a.x+t*dx; p.y=a.y+t*dy; return distance(p,center);
}
Result rrt(const Point *samples,int count){
    Result result={0}; Point center={2,0},goal={4,0};
    result.count=1; result.parent[0]=-1;
    for(int k=0;k<count && result.count<256;k++){
        int near=0; double nearest=distance(result.nodes[0],samples[k]);
        for(int i=1;i<result.count;i++){double d=distance(result.nodes[i],samples[k]);if(d<nearest){nearest=d;near=i;}}
        Point a=result.nodes[near],b=a; Event event={0};
        event.s=samples[k];event.a=a;
        if(nearest==0){event.b=a;event.d=segment_distance(a,a,center);event.status="DUPLICATE";result.log[result.events++]=event;continue;}
        double fraction=fmin(1,2/nearest);b.x+=fraction*(samples[k].x-a.x);b.y+=fraction*(samples[k].y-a.y);
        event.b=b;event.d=segment_distance(a,b,center);
        int accepted=event.d>1 && b.x>=-1 && b.x<=5 && b.y>=-1 && b.y<=3;
        event.status=accepted?"ACCEPT":"REJECT";result.log[result.events++]=event;
        if(accepted){int id=result.count++;result.nodes[id]=b;result.parent[id]=near;
            if(distance(b,goal)<1e-9){int cursor=id;Point reversed[256];
                while(cursor>=0){reversed[result.length++]=result.nodes[cursor];cursor=result.parent[cursor];}
                for(int i=0;i<result.length;i++)result.path[i]=reversed[result.length-1-i];return result;}
        }
    }return result;
}
void update_descendants(int node,int *parent,double *cost,const double *edge,int n){
    for(int i=0;i<n;i++)if(parent[i]==node){cost[i]=cost[node]+edge[i];update_descendants(i,parent,cost,edge,n);}
}
int rewire(int node,int new_parent,double link,int *parent,double *cost,double *edge,int n){
    for(int i=new_parent;i>=0;i=parent[i])if(i==node)return 0;
    parent[node]=new_parent;edge[node]=link;cost[node]=cost[new_parent]+link;update_descendants(node,parent,cost,edge,n);return 1;
}
int coverage(const int *grid,int rows,int cols,Cell *out){
    int count=0;for(int r=0;r<rows;r++)for(int i=0;i<cols;i++){int c=r%2?cols-1-i:i;if(!grid[r*cols+c]){out[count].r=r;out[count++].c=c;}}return count;
}
int connect(const int *grid,int rows,int cols,Cell a,Cell b,Cell *out){
    int queue[256],parent[256],head=0,tail=0,target=b.r*cols+b.c,start=a.r*cols+a.c;
    const int dr[]={-1,0,0,1},dc[]={0,-1,1,0};
    for(int i=0;i<rows*cols;i++)parent[i]=-2;
    queue[tail++]=start;parent[start]=-1;
    while(head<tail){int cur=queue[head++];if(cur==target){int reversed[256],n=0;
        for(int p=cur;p>=0;p=parent[p])reversed[n++]=p;
        for(int i=0;i<n;i++){int p=reversed[n-1-i];out[i].r=p/cols;out[i].c=p%cols;}return n;}
        for(int k=0;k<4;k++){int r=cur/cols+dr[k],c=cur%cols+dc[k],p=r*cols+c;
            if(r>=0&&r<rows&&c>=0&&c<cols&&!grid[p]&&parent[p]==-2){parent[p]=cur;queue[tail++]=p;}}
    }return 0;
}
void print_cells(const char *name,const Cell *cells,int count){printf("%s,",name);for(int i=0;i<count;i++)printf("%s%d:%d",i?";":"",cells[i].r,cells[i].c);puts("");}
int main(void){
    const Point samples[]={{4,0},{0,2},{2,2},{4,2},{4,0}};
    Result r=rrt(samples,5);puts("RRT,step,sx,sy,ax,ay,bx,by,d,status");
    for(int i=0;i<r.events;i++){Event e=r.log[i];printf("RRT,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%s\n",i+1,e.s.x,e.s.y,e.a.x,e.a.y,e.b.x,e.b.y,e.d,e.status);}
    double length=0;for(int i=0;i<r.length;i++){printf("PATH,%d,%.9f,%.9f\n",i,r.path[i].x,r.path[i].y);if(i)length+=distance(r.path[i-1],r.path[i]);}
    printf("LENGTH,%.9f\n",length);
    Point a={0,0},b={4,0},center={2,1},zero={2,0};
    printf("TANGENT,%.9f,COLLISION\n",segment_distance(a,b,center));
    int parent[]={-1,0,0,1,3,2};double costs[]={0,7,4,10,11,6},edge[]={0,7,4,3,1,2};
    int chosen=costs[1]+1<costs[2]+2?1:2;parent[5]=chosen;edge[5]=chosen==1?1:2;costs[5]=costs[chosen]+edge[5];
    assert(rewire(3,5,1,parent,costs,edge,6));
    printf("REWIRE,%d,%.9f,%.9f,%.9f\n",parent[5],costs[5],costs[3],costs[4]);
    printf("WINDOW,%.9f,%.9f,%.9f,%.9f\n",fmax(0,.4-2*.1),fmin(1,.4+2*.1),fmax(-1,-3*.1),fmin(1,3*.1));
    printf("BRAKE,%.9f,REJECT\n",.6*.6/2);
    const double v[]={.5,.5,.3},w[]={0,.3,0},d[]={2,1.5,2.5};double best_score=-1;int best=-1;
    puts("DWA,index,v,w,x,y,theta,H,V,C,G");
    for(int i=0;i<3;i++){double x=v[i]*.1,y=0,theta=w[i]*.1,delta=atan2(-y,2-x)-theta;
        delta=atan2(sin(delta),cos(delta));double h=1/(1+fabs(delta)),c=fmin(d[i]/3,1),g=.4*h+.2*v[i]+.4*c;
        printf("DWA,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",i,v[i],w[i],x,y,theta,h,v[i],c,g);
        if(g>best_score){best_score=g;best=i;}}
    printf("BEST,%d,%.9f,%.9f\n",best,v[best],w[best]);
    const int grid1[]={0,0,0,0,1,0},grid2[]={0,1,0,0,1,0};Cell visits[256],path[256],part[256];
    int n=coverage(grid1,2,3,visits);print_cells("VISITS1",visits,n);int length_path=1;path[0]=visits[0];
    for(int i=1;i<n;i++){int m=connect(grid1,2,3,path[length_path-1],visits[i],part);assert(m);for(int j=1;j<m;j++)path[length_path++]=part[j];}
    print_cells("TRAVEL1",path,length_path);n=coverage(grid2,3,2,visits);print_cells("VISITS2",visits,n);
    const Point duplicate[]={{0,0}};const int blocked[]={1,1},disconnected[]={0,1,0};Cell ca={0,0},cb={0,2};
    assert(r.length==5&&fabs(length-8)<1e-12&&costs[3]==7&&costs[4]==8);
    assert(segment_distance(center,center,zero)==1&&segment_distance(a,b,center)==1);
    assert(rrt(duplicate,1).count==1&&rrt(samples,1).length==0);
    assert(best==2&&fabs(best_score-.7933333333333333)<1e-12&&.6*.6/2>.15);
    assert(coverage(blocked,1,2,visits)==0&&connect(disconnected,1,3,ca,cb,part)==0&&length_path==8);
    assert(!rewire(5,4,1,parent,costs,edge,6));
    puts("CHECKS,tangent,zero-length,duplicate,not-found,no-safe-speed,blocked-grid,disconnected,PASS");return 0;
}
