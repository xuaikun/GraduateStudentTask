#include <iostream>
#include <memory.h>
#include <stdio.h>
using namespace std;

const int maxnum=101;
bool array[maxnum][maxnum];
bool use[maxnum]; //进入团的标号
int cn,bestn,p,e;

void dfs(int i)
{
    int j;
    bool flag;

    if(i>p)
    {
        bestn=cn;
        printf("%d\n",bestn);
        for(j=1;j<=p;j++)
            if(use[j])
                printf("%d ",j);
        printf("\n");
        return ;
    }

    flag=true;
    for(j=1;j<i;j++)
        if(use[j]&&!array[j][i])
        {
            flag=false;
            break;
        }
    if(flag)
    {
        cn++;
        use[i]=true;
        dfs(i+1);
        cn--;
    }
    if(cn+p-i>bestn)  //剪枝
    {
        use[i]=false;
        dfs(i+1);
    }
}

int main()
{
    int num,i,u,v;
    scanf("%d",&num);
    while(num--)
    {
        memset(array,false,sizeof(array));
        memset(use,false,sizeof(use));
        scanf("%d%d",&p,&e);
        for(i=0;i<e;i++)
        {
            scanf("%d%d",&u,&v);
            array[u][v]=true; 
            array[v][u]=true;
        }

        cn=bestn=0;
        dfs(1);
        //printf("%d\n",bestn);
    }

    return 0;
}
 
/*
1
5 7
1 2
1 4
1 5
2 3
2 5
3 5
4 5
*/
