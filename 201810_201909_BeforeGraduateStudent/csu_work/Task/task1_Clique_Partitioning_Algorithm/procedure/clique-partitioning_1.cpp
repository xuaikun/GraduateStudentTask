#include <iostream>
#include<cstdio>
#include<algorithm>
#include<queue>
#include<cstring>
#include <stdlib.h>
#include <time.h> 
using namespace std;
struct node//����ṹ��
{
    int last;//������ĵ�
    int v[10];//v[i]=1��ʾ����i�Ѿ��ڸü�����v[i]=0���෴
    int num;//�������
    node()//���캯������ʼ��
    {
        memset(v,0,sizeof(v));
        num=0;
    }
};
int ans,n;
int a[11][11];
queue<node*>q;
void bfs(node *head)//��֧�����㷨
{
    while(!q.empty()) q.pop();
    q.push(head);
    int i,j;
    while(!q.empty())
    {
        node *tem=q.front();//ȡ�Զ�
        ans=max(ans,tem->num);
        if(ans==n)break;
        q.pop();
        int t=tem->last;
        for(i=1; i<=n; i++)//�������еĶ���
        {
            if(i==t)continue;
            if(a[t][i]==1&&tem->v[i]==0)//����õ㻹δ����ü��ϲ��Ҹõ���������ĵ�������
            {
                for(j=1; j<=n; j++)//�жϸõ��Ƿ����Ѿ�����ü��ϵ�ÿһ����������
                {
                    if(tem->v[j]==1&&a[i][j]==0)break;
                }
                if(j>n)//����õ���ü��ϵ�ÿһ���������ߣ��ڸü����м���õ�
                {
                    node *nod=new node();
                    for(int k=1; k<=n; k++)//��ʼ��
                    {
                        nod->v[k]=tem->v[k];
                    }
                    nod->last=i;
                    nod->v[i]=1;
                    nod->num=tem->num+1;
                    q.push(nod);//�½ڵ�����ö���
                }
            }
 
        }
    }
}
int main()
{
    int m,i,j,k;
    srand((unsigned)time(0));
    cout<<"�����붥�����:";
    cin>>n;
    //n = 5;
   // cout << n << endl;
    for(i=1; i<=n; i++)
    {
        //cout<<"��������� "<<i<<" ��ص����еĵ���������:";
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
    //for(i=0; i<=n; i++)//���ڵ������еĵ�����
      //  a[i][0]=a[0][i]=1;
    node *head=new node();
    head->last=0;
    ans=0;
    bfs(head);
    cout<<"����ŵĸ���Ϊ��";
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

