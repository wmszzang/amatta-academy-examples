/* EP.23: 최소 힙의 비교 키를 (f,g,행,열)로 통일한다. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

typedef struct { int f,g,r,c; } Node;
typedef struct { Node *a; int size, capacity; } Heap;
static int less(Node a, Node b) {
    if(a.f!=b.f) return a.f<b.f;
    if(a.g!=b.g) return a.g<b.g;
    if(a.r!=b.r) return a.r<b.r;
    return a.c<b.c;
}
static void push(Heap *h, Node n) {
    if(h->size==h->capacity) {
        int capacity=h->capacity ? 2*h->capacity : 16;
        Node *p=(Node*)realloc(h->a,sizeof(Node)*capacity);
        if(!p) { perror("heap"); exit(1); }
        h->a=p; h->capacity=capacity;
    }
    int i=h->size++;
    while(i>0 && less(n,h->a[(i-1)/2])) { h->a[i]=h->a[(i-1)/2]; i=(i-1)/2; }
    h->a[i]=n;
}
static Node pop(Heap *h) {
    Node out=h->a[0], last=h->a[--h->size]; int i=0;
    while(2*i+1<h->size) {
        int k=2*i+1;
        if(k+1<h->size && less(h->a[k+1],h->a[k])) ++k;
        if(!less(h->a[k],last)) break;
        h->a[i]=h->a[k]; i=k;
    }
    if(h->size) h->a[i]=last;
    return out;
}
static void search(const char *name,const int *grid,int rows,int cols,int terrain,int heuristic,FILE *pf,FILE *tf) {
    int n=rows*cols, *g=(int*)malloc(n*sizeof(int)), *parent=(int*)malloc(n*sizeof(int));
    int *path=(int*)malloc(n*sizeof(int)), count=0, found=0;
    Heap heap={NULL,0,0};
    if(!g||!parent||!path) { perror("search"); exit(1); }
    for(int i=0;i<n;++i) { g[i]=INT_MAX; parent[i]=-1; }
    g[0]=terrain?grid[0]:0;
    if(terrain || (!grid[0]&&!grid[n-1])) {
        Node first={g[0]+(heuristic&&!terrain?rows+cols-2:0),g[0],0,0}; push(&heap,first);
    }
    while(heap.size) {
        Node cur=pop(&heap); int id=cur.r*cols+cur.c;
        if(cur.g!=g[id]) continue;
        fprintf(tf,"%s,%d,%d,%d,%d,%d,%d\n",name,count++,cur.r,cur.c,cur.g,cur.f-cur.g,cur.f);
        if(id==n-1) { found=1; break; }
        const int dr[]={1,-1,0,0},dc[]={0,0,1,-1};
        for(int j=0;j<4;++j) {
            int r=cur.r+dr[j],c=cur.c+dc[j];
            if(r<0||r>=rows||c<0||c>=cols) continue;
            int ni=r*cols+c;
            if(!terrain&&grid[ni]) continue;
            int ng=cur.g+(terrain?grid[ni]:1);
            if(ng<g[ni]) {
                g[ni]=ng; parent[ni]=id;
                int h=heuristic&&!terrain?rows-1-r+cols-1-c:0;
                Node next={ng+h,ng,r,c}; push(&heap,next);
            }
        }
    }
    if(found) {
        int length=0;
        for(int i=n-1;i>=0;i=parent[i]) path[length++]=i;
        printf("%s: cost=%d points=%d moves=%d popped=%d\n",name,g[n-1],length,length-1,count);
        for(int i=0;i<length;++i) { int id=path[length-1-i]; fprintf(pf,"%s,%d,%d,%d\n",name,i,id/cols,id%cols); }
    } else printf("%s: NO PATH\n",name);
    free(heap.a); free(g); free(parent); free(path);
}
static void distance_case(FILE *f,const char *name,double bx,double by,double px,double py,double radius) {
    double denominator=bx*bx+by*by,raw=0,t=0;
    if(denominator!=0) { raw=(px*bx+py*by)/denominator; t=fmax(0,fmin(1,raw)); }
    double qx=t*bx,qy=t*by,d=hypot(px-qx,py-qy);
    const char *result=radius<0?"-":(d<=radius?"COLLISION":"SAFE");
    fprintf(f,"%s,",name);
    if(denominator==0) fprintf(f,"NA"); else fprintf(f,"%.4f",raw);
    fprintf(f,",%.4f,%.4f,%.4f,%.4f,%s\n",t,qx,qy,d,result);
    printf("%s: d=%.4f %s\n",name,d,result);
}
int main(void) {
    FILE *pf=fopen("paths.csv","wb"),*tf=fopen("trace.csv","wb"),*df=fopen("distances.csv","wb");
    if(!pf||!tf||!df) { perror("output"); return 1; }
    fprintf(pf,"case,step,row,col\n"); fprintf(tf,"case,step,row,col,g,h,f\n");
    const int terrain[]={1,3,1,1,5,1,4,2,1},grid[]={0,0,0,0,1,1,0,1,0,0,0,0,0,1,1,0};
    const int blocked[]={0,0,0,0,1,1,1,1,0,0,0,0,0,1,1,0},one[]={7},zero[]={0};
    const int extra1[]={1,2,1,1},extra2[]={1,9,9,1,9,1,1,1,1};
    search("P1",terrain,3,3,1,0,pf,tf); search("P2",grid,4,4,0,1,pf,tf); search("P2_h0",grid,4,4,0,0,pf,tf);
    search("NO_PATH",blocked,4,4,0,1,pf,tf); search("ONE_TERRAIN",one,1,1,1,0,pf,tf); search("ONE_GRID",zero,1,1,0,1,pf,tf);
    search("P1_EXTRA1",extra1,2,2,1,0,pf,tf); search("P1_EXTRA2",extra2,3,3,1,0,pf,tf);
    fprintf(df,"case,t_raw,t,qx,qy,distance,result\n");
    distance_case(df,"P3_IN",4,0,2,3,-1); distance_case(df,"P3_AFTER",4,0,6,0,-1);
    distance_case(df,"P3_BEFORE",4,0,-1,-1,-1); distance_case(df,"P3_ZERO",0,0,2,3,-1);
    distance_case(df,"P4_TOUCH",4,0,2,1,1); distance_case(df,"P4_SAFE",4,0,2,2,1);
    fclose(pf); fclose(tf); fclose(df); return 0;
}
