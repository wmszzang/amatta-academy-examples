// C 예제와 같은 비교 순서를 표준 우선순위 큐로 구현한다.
#include <cstdio>
#include <cmath>
#include <queue>
#include <tuple>
#include <vector>
#include <functional>
#include <climits>
using namespace std;
void search(const char* name, vector<vector<int>> a, bool terrain, bool heuristic, FILE* pf, FILE* tf) {
    int nr=a.size(), nc=a[0].size(), n=nr*nc, count=0; bool found=false;
    vector<int> g(n,INT_MAX), parent(n,-1), path;
    using Node=tuple<int,int,int,int>;
    priority_queue<Node,vector<Node>,greater<Node>> heap;
    auto h=[&](int r,int c){return heuristic&&!terrain?nr-1-r+nc-1-c:0;};
    g[0]=terrain?a[0][0]:0;
    if(terrain||(!a[0][0]&&!a[nr-1][nc-1])) heap.emplace(g[0]+h(0,0),g[0],0,0);
    while(!heap.empty()) {
        int f,cost,r,c; tie(f,cost,r,c)=heap.top(); heap.pop(); int id=r*nc+c;
        if(cost!=g[id]) continue;
        fprintf(tf,"%s,%d,%d,%d,%d,%d,%d\n",name,count++,r,c,cost,f-cost,f);
        if(id==n-1){found=true;break;}
        int dr[]={1,-1,0,0},dc[]={0,0,1,-1};
        for(int i=0;i<4;++i){
            int rr=r+dr[i],cc=c+dc[i];
            if(rr<0||rr>=nr||cc<0||cc>=nc||(!terrain&&a[rr][cc]))continue;
            int next=rr*nc+cc, ng=cost+(terrain?a[rr][cc]:1);
            if(ng<g[next]){g[next]=ng;parent[next]=id;heap.emplace(ng+h(rr,cc),ng,rr,cc);}
        }
    }
    if(found){
        for(int id=n-1;id>=0;id=parent[id])path.push_back(id);
        printf("%s: cost=%d points=%d moves=%d popped=%d\n",name,g[n-1],int(path.size()),int(path.size())-1,count);
        for(int i=0;i<int(path.size());++i){int id=path[path.size()-1-i];fprintf(pf,"%s,%d,%d,%d\n",name,i,id/nc,id%nc);}
    }else printf("%s: NO PATH\n",name);
}
void distance_case(FILE* file,const char* name,double bx,double by,double px,double py,double radius){
    double den=bx*bx+by*by,raw=0,t=0;
    if(den!=0){raw=(px*bx+py*by)/den;t=fmax(0,fmin(1,raw));}
    double qx=t*bx,qy=t*by,d=hypot(px-qx,py-qy);
    const char* result=radius<0?"-":(d<=radius?"COLLISION":"SAFE");
    fprintf(file,"%s,",name);
    if(den==0)fprintf(file,"NA");else fprintf(file,"%.4f",raw);
    fprintf(file,",%.4f,%.4f,%.4f,%.4f,%s\n",t,qx,qy,d,result);
    printf("%s: d=%.4f %s\n",name,d,result);
}
int main(){
    FILE* pf=fopen("paths.csv","wb");FILE* tf=fopen("trace.csv","wb");FILE* df=fopen("distances.csv","wb");
    if(!pf||!tf||!df){perror("output");return 1;}
    fprintf(pf,"case,step,row,col\n");fprintf(tf,"case,step,row,col,g,h,f\n");
    vector<vector<int>> a={{1,3,1},{1,5,1},{4,2,1}},b={{0,0,0,0},{1,1,0,1},{0,0,0,0},{0,1,1,0}};
    search("P1",a,true,false,pf,tf);search("P2",b,false,true,pf,tf);search("P2_h0",b,false,false,pf,tf);
    b[1][2]=1;search("NO_PATH",b,false,true,pf,tf);search("ONE_TERRAIN",{{7}},true,false,pf,tf);search("ONE_GRID",{{0}},false,true,pf,tf);
    search("P1_EXTRA1",{{1,2},{1,1}},true,false,pf,tf);search("P1_EXTRA2",{{1,9,9},{1,9,1},{1,1,1}},true,false,pf,tf);
    fprintf(df,"case,t_raw,t,qx,qy,distance,result\n");
    distance_case(df,"P3_IN",4,0,2,3,-1);distance_case(df,"P3_AFTER",4,0,6,0,-1);
    distance_case(df,"P3_BEFORE",4,0,-1,-1,-1);distance_case(df,"P3_ZERO",0,0,2,3,-1);
    distance_case(df,"P4_TOUCH",4,0,2,1,1);distance_case(df,"P4_SAFE",4,0,2,2,1);
    fclose(pf);fclose(tf);fclose(df);
}
