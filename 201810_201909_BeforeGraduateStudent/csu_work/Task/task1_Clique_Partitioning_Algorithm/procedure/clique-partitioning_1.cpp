#include <iostream>
#include<cstdio>
#include<algorithm>
#include<queue>
#include<cstring>
#include <stdlib.h>
#include <time.h> 
using namespace std;
struct node//定义结构体
{
    int last;//最后加入的点
    int v[10];//v[i]=1表示顶点i已经在该集合中v[i]=0则相反
    int num;//顶点个数
    node()//构造函数，初始化
    {
        memset(v,0,sizeof(v));
        num=0;
    }
};
int ans,n;
int a[11][11];
queue<node*>q;
void bfs(node *head)//分支界限算法
{
    while(!q.empty()) q.pop();
    q.push(head);
    int i,j;
    while(!q.empty())
    {
        node *tem=q.front();//取对顶
        ans=max(ans,tem->num);
        if(ans==n)break;
        q.pop();
        int t=tem->last;
        for(i=1; i<=n; i++)//搜索所有的顶点
        {
            if(i==t)continue;
            if(a[t][i]==1&&tem->v[i]==0)//如果该点还未加入该集合并且该点与最后加入的点有连线
            {
                for(j=1; j<=n; j++)//判断该点是否与已经加入该集合的每一个点有连线
                {
                    if(tem->v[j]==1&&a[i][j]==0)break;
                }
                if(j>n)//如果该点与该集合的每一个点有连线，在该集合中加入该点
                {
                    node *nod=new node();
                    for(int k=1; k<=n; k++)//初始化
                    {
                        nod->v[k]=tem->v[k];
                    }
                    nod->last=i;
                    nod->v[i]=1;
                    nod->num=tem->num+1;
                    q.push(nod);//新节点推入该队列
                }
            }
 
        }
    }
}
int main()
{
    int m,i,j,k;
    srand((unsigned)time(0));
    cout<<"请输入顶点个数:";
    cin>>n;
    //n = 5;
   // cout << n << endl;
    for(i=1; i<=n; i++)
    {
        //cout<<"请输入与点 "<<i<<" 相关的所有的点的连线情况:";
        for(j=1; j<=n; j++)
        {
            //cin>>a[i][j];
            if(i == j)
            {
            	a[i][j] = 0; 
			}
			else
			{
		        a[i][j] = rand()%2;
		       
			    a[j][i] = a[i][j];
			}
        }
    }
    for(int i = 1; i <= n; i++)
	{
		for(int j = 1; j <= n; j++)
		{ 
		
			cout << a[i][j] << " "; 
			if(j == n)
				cout << endl;		
		}
	}
    //for(i=0; i<=n; i++)//根节点与所有的点相连
      //  a[i][0]=a[0][i]=1;
    node *head=new node();
    head->last=0;
    ans=0;
    bfs(head);
    cout<<"最大团的个数为：";
    cout<<ans<<endl;
    return 0;
}
/*
5
0 1 0 1 1
1 0 1 0 1
0 1 0 0 1
1 0 0 0 1
1 1 1 1 0
*/

