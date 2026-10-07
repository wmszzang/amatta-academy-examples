/* EP.25: C와 C++에서 같은 수치 규약으로 실행 가능한 계산 예제. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>

typedef struct { double x,y; } Point;
typedef struct { Point point; int parent; double cost; } Node;
typedef struct { int r,c; } Cell;
typedef struct { int path[64],count,cost,pops[64],popcost[64],npop; } GridResult;
typedef struct { Node nodes[64]; int count,path[64],np; } Tree;
static const int dr[4]={-1,0,0,1},dc[4]={0,-1,1,0};
static const Point samples[7]={{4,0},{0,2},{2,2},{4,2},{4,0},{4,-1},{2,1.2}};
static const Point center={2,0};
static const int costgrid[9]={1,3,1,1,5,1,4,2,1};
static const int mapgrid[16]={0,0,0,0,1,1,0,1,0,0,0,0,0,1,1,0};
static double distance(Point a,Point b){return hypot(a.x-b.x,a.y-b.y);}
static double segment(Point a,Point b,Point c){
    double ux=b.x-a.x,uy=b.y-a.y,den=ux*ux+uy*uy;
    double t=den?((c.x-a.x)*ux+(c.y-a.y)*uy)/den:0;
    Point q;
    if(t<0)t=0; if(t>1)t=1;
    q.x=a.x+t*ux; q.y=a.y+t*uy;
    return distance(q,c);
}
static int safe(Point a,Point b){return segment(a,b,center)>1.;}
static GridResult search(const int *grid,int rows,int cols,int start,int goal,int weighted){
    GridResult out={0}; int best[64],parent[64],closed[64]={0},n=rows*cols,i;
    out.cost=-1;
    if(!weighted&&(grid[start]||grid[goal]))return out;
    for(i=0;i<n;i++){best[i]=1000000;parent[i]=-1;}
    best[start]=weighted?grid[start]:0;
    for(;;){
        int current=-1,bestf=1000000,bestg=1000000;
        for(i=0;i<n;i++)if(!closed[i]&&best[i]<1000000){
            int f=best[i]+(weighted?0:abs(i/cols-goal/cols)+abs(i%cols-goal%cols));
            if(f<bestf||(f==bestf&&best[i]<bestg)) {current=i;bestf=f;bestg=best[i];}
        }
        if(current<0)return out;
        closed[current]=1;out.pops[out.npop]=current;out.popcost[out.npop++]=best[current];
        if(current==goal){
            int reverse[64],count=0,j;
            for(i=goal;i>=0;i=parent[i])reverse[count++]=i;
            for(j=0;j<count;j++)out.path[j]=reverse[count-1-j];
            out.count=count;out.cost=best[goal];return out;
        }
        for(i=0;i<4;i++){
            int r=current/cols+dr[i],c=current%cols+dc[i],next,candidate;
            if(r<0||r>=rows||c<0||c>=cols)continue;
            next=r*cols+c;if(!weighted&&grid[next])continue;
            candidate=best[current]+(weighted?grid[next]:1);
            if(candidate<best[next]){best[next]=candidate;parent[next]=current;}
        }
    }
}
static int ancestor(const Tree *tree,int child,int target){
    while(child>=0){if(child==target)return 1;child=tree->nodes[child].parent;}return 0;
}
static void propagate(Tree *tree,int index,int emit){
    int i;for(i=0;i<tree->count;i++)if(tree->nodes[i].parent==index){
        tree->nodes[i].cost=tree->nodes[index].cost+distance(tree->nodes[index].point,tree->nodes[i].point);
        if(emit)printf("RRTSTAR,propagate,%d,%d,%.9f\n",i,index,tree->nodes[i].cost);
        propagate(tree,i,emit);
    }
}
static Tree sampling(const Point *input,int size,int improve,int emit){
    Tree tree={0};int s,i;tree.count=1;tree.nodes[0].parent=-1;
    for(s=0;s<size;s++){
        int nearest=0,parent,index,duplicate=0;double nearestDistance=distance(tree.nodes[0].point,input[s]),ratio,cost;
        Point point,origin;
        for(i=1;i<tree.count;i++){double d=distance(tree.nodes[i].point,input[s]);if(d<nearestDistance){nearest=i;nearestDistance=d;}}
        if(nearestDistance==0){if(emit)puts("RRTSTAR,duplicate,-1,-1,0.000000000");continue;}
        origin=tree.nodes[nearest].point;ratio=fmin(1,3/nearestDistance);
        point.x=origin.x+ratio*(input[s].x-origin.x);point.y=origin.y+ratio*(input[s].y-origin.y);
        if(point.x< -1||point.x>5||point.y< -1||point.y>3||!safe(origin,point)){
            if(emit)puts("RRTSTAR,reject,-1,-1,0.000000000");continue;
        }
        for(i=0;i<tree.count;i++)if(distance(tree.nodes[i].point,point)<1e-12)duplicate=1;
        if(duplicate){if(emit)puts("RRTSTAR,duplicate,-1,-1,0.000000000");continue;}
        parent=nearest;cost=tree.nodes[parent].cost+distance(tree.nodes[parent].point,point);
        if(improve){
            parent=-1;cost=1e100;
            for(i=0;i<tree.count;i++){
                double d=distance(tree.nodes[i].point,point),candidate=tree.nodes[i].cost+d;
                if(d<=3&&safe(tree.nodes[i].point,point)&&candidate<cost){parent=i;cost=candidate;}
            }
        }
        assert(parent>=0);index=tree.count++;
        tree.nodes[index].point=point;tree.nodes[index].parent=parent;tree.nodes[index].cost=cost;
        if(emit)printf("RRTSTAR,insert,%d,%d,%.9f\n",index,parent,cost);
        if(improve)for(i=0;i<index;i++){
            double d=distance(point,tree.nodes[i].point),candidate=cost+d;
            if(d<=3&&candidate<tree.nodes[i].cost&&!ancestor(&tree,index,i)&&safe(point,tree.nodes[i].point)){
                tree.nodes[i].parent=index;tree.nodes[i].cost=candidate;
                if(emit)printf("RRTSTAR,rewire,%d,%d,%.9f\n",i,index,candidate);
                propagate(&tree,i,emit);
            }
        }
    }
    for(i=0;i<tree.count;i++){Point goal={4,0};if(distance(tree.nodes[i].point,goal)<1e-12){
        int reverse[64],count=0,k=i,j;while(k>=0){reverse[count++]=k;k=tree.nodes[k].parent;}
        for(j=0;j<count;j++)tree.path[j]=reverse[count-1-j];tree.np=count;break;
    }}
    return tree;
}
static int evaluate(int count,int emit){
    const double v[3]={.5,.5,.3},w[3]={0,.3,0},d[3]={2,1.5,2.5};
    double best=-1;int chosen=-1,i;
    for(i=0;i<count;i++){
        double x=v[i]*.1,angle=w[i]*.1,error=atan2(sin(-angle),cos(-angle));
        double h=1/(1+fabs(error)),clearance=fmin(d[i]/3,1),score=.4*h+.2*v[i]+.4*clearance;
        if(emit)printf("DWA,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",v[i],w[i],x,0.,angle,h,v[i],clearance,score);
        if(score>best){best=score;chosen=i;}
    }return chosen;
}
static int bfs(const int *grid,int rows,int cols,int start,int goal,int *path){
    int queue[64],front=0,back=1,parent[64],seen[64]={0},i;queue[0]=start;seen[start]=1;
    for(i=0;i<64;i++)parent[i]=-1;
    while(front<back){int here=queue[front++];if(here==goal){
        int reverse[64],count=0;for(i=goal;i>=0;i=parent[i])reverse[count++]=i;
        for(i=0;i<count;i++)path[i]=reverse[count-1-i];return count;
    }
    for(i=0;i<4;i++){int r=here/cols+dr[i],c=here%cols+dc[i],next;
        if(r<0||r>=rows||c<0||c>=cols)continue;next=r*cols+c;
        if(!seen[next]&&!grid[next]){seen[next]=1;parent[next]=here;queue[back++]=next;}
    }}return 0;
}
static void coverage(const int *grid,int rows,int cols,int fixture,int emit,int *counts){
    int order[64],route[512],missing[64],no=0,nr=0,nm=0,r,j,i;
    for(r=0;r<rows;r++)for(j=0;j<cols;j++){int c=r%2?cols-1-j:j;if(!grid[r*cols+c])order[no++]=r*cols+c;}
    if(no)route[nr++]=order[0];
    for(i=1;i<no;i++){int part[64],count=bfs(grid,rows,cols,route[nr-1],order[i],part);
        if(!count)missing[nm++]=order[i];else for(j=1;j<count;j++)route[nr++]=part[j];}
    counts[0]=no;counts[1]=nr;counts[2]=nm;
    if(emit){const char *labels[3]={"order","route","unreachable"};int *lists[3]={order,route,missing};
        for(j=0;j<3;j++)for(i=0;i<counts[j];i++)printf("COVERAGE,%d,%s,%d,%d\n",fixture,labels[j],lists[j][i]/cols,lists[j][i]%cols);}
}
static void verify(void){
    GridResult d=search(costgrid,3,3,0,8,1),a=search(mapgrid,4,4,0,15,0);
    int blocked[16],single[1]={0},wall[1]={1},split[3]={0,1,0},counts[3],i,sum=0;
    Tree tree=sampling(samples,7,1,0),basic=sampling(samples,5,0,0),missing=sampling(samples,1,1,0);
    Point zero={0,0},p={3,4},end={4,0},touch={2,1};Point duplicate[8];double length=0;
    assert(d.cost==7&&a.cost==6&&a.count==7);
    for(i=0;i<d.count;i++)sum+=costgrid[d.path[i]];assert(sum==d.cost);
    for(i=1;i<a.count;i++)assert(abs(a.path[i]/4-a.path[i-1]/4)+abs(a.path[i]%4-a.path[i-1]%4)==1);
    memcpy(blocked,mapgrid,sizeof(blocked));blocked[6]=1;assert(search(blocked,4,4,0,15,0).count==0);
    assert(search(single,1,1,0,0,0).cost==0);
    assert(segment(zero,zero,p)==5&&segment(zero,end,touch)==1);
    assert(tree.np==3&&tree.path[1]==6&&fabs(tree.nodes[4].cost-4.66476151587624)<1e-9);
    assert(fabs(tree.nodes[5].cost-5.66476151587624)<1e-9&&basic.nodes[4].cost==8&&missing.np==0);
    for(i=1;i<tree.np;i++){Point x=tree.nodes[tree.path[i-1]].point,y=tree.nodes[tree.path[i]].point;assert(safe(x,y));length+=distance(x,y);}
    assert(fabs(length-tree.nodes[4].cost)<1e-12);
    memcpy(duplicate,samples,sizeof(samples));duplicate[7]=samples[6];assert(sampling(duplicate,8,1,0).count==7);
    assert(evaluate(3,0)==2&&evaluate(0,0)==-1);
    coverage(wall,1,1,0,0,counts);assert(counts[0]==0&&counts[1]==0);
    coverage(split,1,3,0,0,counts);assert(counts[2]==1);
}
#ifndef PATH_WRAPUP_LIBRARY
int main(void){
    int i,j,counts[3],g0[6]={0,0,0,0,1,0},g1[6]={0,1,0,0,1,0},g2[3]={0,1,0};Tree tree,basic;
    verify();
    for(j=0;j<2;j++){GridResult result=search(j?mapgrid:costgrid,j?4:3,j?4:3,0,j?15:8,!j);int cols=j?4:3;const char *label=j?"ASTAR":"DIJKSTRA";
        printf("%s,cost,%.9f\n",label,(double)result.cost);
        for(i=0;i<result.npop;i++)printf("%s,pop,%d,%d,%.9f\n",label,result.pops[i]/cols,result.pops[i]%cols,(double)result.popcost[i]);
        for(i=0;i<result.count;i++)printf("%s,path,%d,%d\n",label,result.path[i]/cols,result.path[i]%cols);
    }
    tree=sampling(samples,7,1,1);
    for(i=0;i<tree.count;i++)printf("RRTSTAR,node,%d,%.9f,%.9f,%d,%.9f\n",i,tree.nodes[i].point.x,tree.nodes[i].point.y,tree.nodes[i].parent,tree.nodes[i].cost);
    printf("RRTSTAR,path");for(i=0;i<tree.np;i++)printf(",%d",tree.path[i]);puts("");
    basic=sampling(samples,5,0,0);printf("RRT,cost,%.9f\n",basic.nodes[4].cost);
    evaluate(3,1);puts("DWA,best,0.300000000,0.000000000");
    coverage(g0,2,3,0,1,counts);coverage(g1,3,2,1,1,counts);coverage(g2,1,3,2,1,counts);
    puts("BOUNDARIES,PASS");return 0;
}
#endif
