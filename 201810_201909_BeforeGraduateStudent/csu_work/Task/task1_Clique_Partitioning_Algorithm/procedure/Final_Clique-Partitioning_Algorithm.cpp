#include<iostream>
#include<cstdio>
#include<cstdlib>
#include<cstring>
#include<cmath>
#include<vector>
#include<algorithm>
#include<queue>
using namespace std;
 
const int maxn=130;
int n,m,ans,ne[maxn],ce[maxn],list[maxn][maxn];
bool g[maxn][maxn];
void dfs(int size)
{
 if(ans>100000)return ;
 int i,j,k,t,cnt,best=0;
 if(ne[size]==ce[size])
    {
     if(ce[size]==0)++ans;
     return ;
    }
 for(t=0,i=1;i<=ne[size];++i)
    {
     for(cnt=0,j=ne[size]+1;j<=ce[size];++j)
        if(!g[list[size][i]][list[size][j]])++cnt;
     if(t==0||cnt<best)t=i,best=cnt;
    }
 if(t&&best<=0)return ;//¼ôÖ¦1
 for(k=ne[size]+1;k<=ce[size];++k)
    {
     if(t>0)
        {
         for(i=k;i<=ce[size];++i)
            if(!g[list[size][t]][list[size][i]])break;
         swap(list[size][k],list[size][i]);//×î´ó»¯¼ôÖ¦1
        }
     i=list[size][k];
     ne[size+1]=ce[size+1]=0;
     for(j=1;j<k;++j)
        if(g[i][list[size][j]])
            list[size+1][++ne[size+1]]=list[size][j];
     for(ce[size+1]=ne[size+1],j=k+1;j<=ce[size];++j)
        if(g[i][list[size][j]])
          list[size+1][++ce[size+1]]=list[size][j];
     dfs(size+1);
     if(ans>100000)return ;
     ++ne[size];
     --best;
     for(j=k+1,cnt=0;j<=ce[size];++j)
        if(!g[i][list[size][j]])
            ++cnt;
     if(t==0||cnt<best)t=k,best=cnt;
     if(t&&best<=0)break;
    }
}
 
void  cluster_count()
{
 int i;
 ne[0]=0;ce[0]=0;
 for(i=1;i<=n;++i)
    list[0][++ce[0]]=i;
 ans=0;
 dfs(0);
}
 
int main()
{
 while(scanf("%d",&n)!=EOF)
    {
     memset(g,0,sizeof(g));
     /*for(int i=0;i<m;i++)
        {
         int a,b;
         scanf("%d%d",&a,&b);
         g[a][b]=g[b][a]=true;
        }*/
       for(int i=1; i<=n; i++)
            for(int j=1; j<=n; j++)
                //scanf("%d", &fuck.G[i][j]);
                {
                	if(i == j)
                	{
                		g[i][j] = 0;
					}
					else{
						g[i][j] = rand()%2;
                		g[j][i] = g[i][j];
					}
                	
				}
		for(int i=1; i<= n; i++)
            for(int j=1; j<= n; j++)
                //scanf("%d", &fuck.G[i][j]);
                {
                	cout << g[i][j] << " ";
                	if(j == n )
                	{
                		cout << endl;
					}
				}  
     cluster_count();
     if(ans>100000)printf("Too many maximal sets of friends.\n");
        else printf("%d\n",ans);
    }
 return 0;
}
