#include <iostream>
#include <cstring>
#include <cstdio>
#include <stdlib.h>
#include <time.h>

using namespace std;

struct MAX_CLIQUE {
    static const int N=100;

    bool G[N][N];
    int n, Max[N], Alt[N][N], ans;

    bool DFS(int cur, int tot) {
        if(cur==0) {
            if(tot>ans) {
                ans=tot;
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
            if(DFS(nxt, tot+1)) return 1;
        }
        return 0;
    }

    int MaxClique() {
        ans=0, memset(Max, 0, sizeof Max);
        for(int i=n-1; i>=0; i--)
		 {
            int cur=0;
            for(int j=i+1; j<n; j++)
			{
				if(G[i][j]) 
				{
					Alt[1][cur++]=j;	
				}	
			} 
            DFS(cur, 1);
            Max[i]=ans;
        }
        return ans;
    }
};

MAX_CLIQUE fuck;

int main() {
	srand((unsigned)time(0));
    while(scanf("%d", &fuck.n), fuck.n) {
        for(int i=0; i<fuck.n; i++)
            for(int j=0; j<fuck.n; j++)
                //scanf("%d", &fuck.G[i][j]);
                {
                	if(i == j)
                	{
                		fuck.G[i][j] = 0;
					}
					else{
						fuck.G[i][j] = rand()%2;
                		fuck.G[j][i] = fuck.G[i][j];
					}
                	
				}
		for(int i=0; i<fuck.n; i++)
            for(int j=0; j<fuck.n; j++)
                //scanf("%d", &fuck.G[i][j]);
                {
                	cout << fuck.G[i][j] << " ";
                	if(j == fuck.n - 1)
                	{
                		cout << endl;
					}
				}
        printf("%d\n", fuck.MaxClique());
    }
    return 0;
}

//ZOJ 1492
