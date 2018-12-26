#include <iostream>
#include <cstring>
#include <cstdio>

using namespace std;

struct MAX_CLIQUE {
    static const int N=106;

    bool G[N][N];
    int Max[N], Alt[N][N], x[N], y[N];
    int n, ans, *path, *res;

    bool DFS(int cur, int tot) {
        if(cur==0) {
            if(tot>ans) {
                swap(path, res), ans=tot;
                return 1;
            }
            return 0;
        }
        for(int i=0; i<cur; i++) {
            if(cur-i+tot<=ans) return 0;
            int u=Alt[tot][i];
            if(Max[u]+tot<=ans) return 0;
            int nxt=0;
            for(int j=i+1; j<cur; j++)
                if(G[u][Alt[tot][j]]) Alt[tot+1][nxt++]=Alt[tot][j];
            path[tot+1]=u;
            if(DFS(nxt, tot+1)) return 1;
        }
        return 0;
    }

    int MaxClique() {
        ans=0, memset(Max, 0, sizeof Max);
        path=x, res=y;
        for(int i=n-1, cur; i>=0; i--) {
            path[1]=i, cur=0;
            for(int j=i+1; j<n; j++) if(G[i][j]) Alt[1][cur++]=j;
            DFS(cur, 1);
            Max[i]=ans;
        }
        return ans;
    }
};

MAX_CLIQUE fuck;

int main() {
    int T, m;
    scanf("%d", &T);
    for(int ca=1; ca<=T; ca++) {
        scanf("%d%d", &fuck.n, &m);
        memset(fuck.G, true, sizeof fuck.G);
        for(int i=0, a, b; i<m; i++) {
            scanf("%d%d", &a, &b);
            fuck.G[a-1][b-1]=fuck.G[b-1][a-1]=0;
        }
        int ans=fuck.MaxClique();
        printf("%d\n", ans);
        for(int i=1; i<=ans; i++) {
            printf("%d", fuck.res[i]+1);
            if(i==ans) printf("\n"); else printf(" ");
        }
    }
    return 0;
}

//POJ 1419
