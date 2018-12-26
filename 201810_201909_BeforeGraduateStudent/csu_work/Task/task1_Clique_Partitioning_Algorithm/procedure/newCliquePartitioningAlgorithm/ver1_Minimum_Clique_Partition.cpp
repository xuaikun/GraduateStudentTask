#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
//#define Debug                              //调试用的 
#define N 1004
using namespace std;

struct {
	int EdgeList;
	int Max_Common_Neighbor;
	int Common_Neighbor;
	int Common_Neighbor_New;
	int Edge_Delete;
	 
}Clique[N][N];

int main()
{
go_on:
	srand((unsigned)time(0));
	int n;                                //定义节点个数 
	int Clique_Count = 0;				  // 分团个数 
	cout << "输入节点个数：";
	cin >> n;                            //输入节点个数 
	int Node[n];                         //用于检验某个节点是否没有加入某个团 
	int NodeList[n][n];                 //检查节点都进入了哪个NodeList，记录生成了几个NodeList 
	int First[n];						//定义两个中间计算矩阵 
	int Second[n];
	int Common_Neighbor_Sort[n];		//Common_Neighbor排序
	int Degree[n];						//每个节点的度 
	int Degree_Inceasing_Order[n];		//把度给排序 
	int Max_Degree = 0;												//将度初始化为1,也是终止条件，当最大度为0时，终止 
	for(int i = 1; i <= n; i++)          //初始化节点 
	{
		Node[i]        = 1;               //确保每个节点都存在 
		NodeList[i][1] = 1;               //确保初始化时每个节点存在于一个Clique中
		for(int j = 2; j <= n; j++)      //每个NodeList初始化时只有NodeList[i][0] = 1，i != 0时 NodeList[i][j] = 0; 
		{
			NodeList[i][j] = 0;
		} 
 	}
 	/*打印节点与团的情况*/
/*#ifdef	Debug
	for(int i = 1; i <= n; i++)
	{
		cout << "Node["     << i << "]="    << Node[i]        << endl;
		cout << "NodeList[" << i << "][0]=" << NodeList[i][1] << endl;
	}
#endif*/ 
	
	for(int i = 1; i <= n; i++)
	{
		for(int j = 1; j <= n; j++)
		{
			if(i == j)
			{
				Clique[i][j].EdgeList = 0;
			}
			else
			{
				Clique[i][j].EdgeList  = rand()%2;
				Clique[j][i].EdgeList  =Clique[i][j].EdgeList;
			}
		}
	}
#ifdef Debug 
	cout << "***************" << endl;
	cout << "print EdgeList " << endl; 
	for(int i = 1; i <= n; i++)
	{
		for(int j = 1; j <= n; j++)
		{
			cout << Clique[i][j].EdgeList;
			if(j == n)
				cout << endl;
		}
	}
	cout << "***************" <<endl;
#endif
int max_Degree_Inceasing_Order = 1;
while(max_Degree_Inceasing_Order)
{
	Max_Degree = 0;
#ifdef Debug
	cout << "初始化max_Degree_Inceasing_Order = " <<max_Degree_Inceasing_Order << endl;
	cout << "初始化打印Clique[][].EdgeList矩阵" << endl;
#endif
	 
#ifdef Debug 
	cout << "***************" << endl;
	cout << "print EdgeList " << endl; 
	for(int i = 1; i <= n; i++)
	{
		for(int j = 1; j <= n; j++)
		{
			cout << Clique[i][j].EdgeList;
			if(j == n)
				cout << endl;
		}
	}
#endif
	/*求度每个节点的度~*/
	for(int i = 1; i <= n; i++)
	{
		Degree[i] = 0;										//每个节点的度初始化为0 
		for(int j = 1; j <= n; j++)
		{
			Degree[i] = Degree[i] + Clique[i][j].EdgeList; 
		}	
		Degree_Inceasing_Order[i] = Degree[i];
	} 
	int temp;
	for(int i = 1; i < n ; i++)
	{
		int max = i;
		for(int j = i + 1; j <= n; j++)
		{
			if(Degree_Inceasing_Order[max] < Degree_Inceasing_Order[j])
			{
				max = j;
			}		
		}
		{
			temp = Degree_Inceasing_Order[max];
			Degree_Inceasing_Order[max] = Degree_Inceasing_Order[i];
			Degree_Inceasing_Order[i]   = temp;
		}
	} 
	
	max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];
#ifdef Debug
	cout << "Degree_Increasing_Order:" << endl;
	for(int i = 1; i <= n; i++)
	{
		cout << "Degree_Increasing_Order[" << i << "]= " << Degree_Inceasing_Order[i] <<endl;
	}
#endif
	/*计算每条边的common neighbor数量*/
	for(int i = 1; i <= n; i++)				
	{
		
		for(int j = 1; j <= n; j++)						//控制行的变化 
		{
			First[j] = Clique[i][j].EdgeList;           //  提取第i行元素 
		
		}	
		
		for(int k = 1; k <= n; k++)						//控制列的变化
		{
		
			if(Clique[i][k].EdgeList == 1)				//保证操作的两个点是相连的 
			{
			
				for(int j = 1; j <= n; j++)			 
				{
					Second[j] = Clique[k][j].EdgeList;  //  提取第k行元素 
				}
			
				int Diff = 0;								//邻居计数 	
				for(int u = 1; u <= n; u++)
				{
					if(First[u] == Second[u] && First[u] != 0)			//相连的两个点 有共同的点（Common_Neignbor）,并且0不算相连 
					{
							Diff = Diff + 1;
					}	
				}	
				Clique[i][k].Common_Neighbor = Diff;    //每条边的Common_Neighbor数量 
			}
			else
				Clique[i][k].Common_Neighbor = 0;	
		}
	} 
#ifdef Debug
	cout << "****************" << endl;
	cout << "Common_Neighbor" << endl;
	/*cout << "全部" << endl;
	for(int i = 1; i <= n; i++)
	{
		for(int j = 1 ; j <= n; j++)
		{
			cout <<Clique[i][j].Common_Neighbor << " ";
			if(j == n)
			{
				cout << endl;		
			} 	
		}	
	} */
	cout << "Clique[i][j].Common_Neighbor_New:上三角" << endl;
#endif
	for(int i = 1; i <= n; i++)				//	只获取Common_Neighbor上三角部分数据，保证Common_Neighbor[][]只保存不同边的个数 
	{
		for(int j = i; j <= n; j++)
		{
#ifdef Debug
			cout <<Clique[i][j].Common_Neighbor << " ";
#endif
			Clique[i][j].Common_Neighbor_New = Clique[i][j].Common_Neighbor; //将Common_Neighbor的上三角提取出来 
			if(j == n)
			{
#ifdef Debug
				cout << endl;	
#endif	
			} 	
		}	
	} 

	for(int i = 1, j = 1; i <= n; i++)
	{
		int max = Clique[i][1].Common_Neighbor_New; 
		for(j = i; j <= n; j++)
		{
			if(max < Clique[i][j].Common_Neighbor_New)
			{
				max = Clique[i][j].Common_Neighbor_New;	//若当前max值比 Clique.Common_Neighbor_New[i][j]小，则替换 max值 
			}
		}
		Common_Neighbor_Sort[i] = max;
		//cout << "Common_Neighbor_Sort[i] = " << Common_Neighbor_Sort[i] <<endl;
	}
	
	for(int i = 1; i < n ; i++)
	{
		int max = i;
		for(int j = i + 1; j <= n; j++)
		{
			if(Common_Neighbor_Sort[max] < Common_Neighbor_Sort[j])
			{
				max = j;
			}		
		}
		{
			temp = Common_Neighbor_Sort[max];
			Common_Neighbor_Sort[max] = Common_Neighbor_Sort[i];
			Common_Neighbor_Sort[i] = temp;
		}
	} 
#ifdef Debug
	cout << "*********************" << endl;
	cout << "Common_Neighbor_Sort:" ;							//已经将最大的Common_Neighbor找出来了 （increasing order） 
	for(int i = 1; i <= n; i++)
	{
		cout << Common_Neighbor_Sort[i] << " ";
	}
	cout << endl;
#endif


/*#ifdef Debug
	for(int i = 1; i <= n; i++)
	{
		cout << "NodeList["<< i << "[1] = " << NodeList[i][1] << endl; 
	}
#endif*/

	/*首先找出Common_Neighbor最大的，如Common_Neighbor最大值唯一，则继续，如最大值不唯一则再比较
	Degree[],有限合并度最大的两个节点*/
#ifdef Debug 
	cout << "****************" << endl;
#endif
	int Max_Common_Neighbor_num = 0; 
	int Max_Common_Neighbor_i_Point[n];
	int Max_Common_Neighbor_j_Point[n];
	int Max_Common_Neighbor_i_Point_Last;
	int Max_Common_Neighbor_j_Point_Last;
	int Max_i;
	int Max_j;											//一点存在 Max_j > Max_i 
	for(int i = 1; i <= n; i++)
	{
		for(int j = i + 1; j <= n; j++)
		{
			// Common_Neighbor数量最大点为 Common_Neighbor_Sort[1]  
			if(Common_Neighbor_Sort[1] == Clique[i][j].Common_Neighbor_New && Clique[i][j].EdgeList != 0)	//找到最大Common_Neighbor的两个节点 
			{
				Max_Common_Neighbor_num++;					 						//统计最大Common_Neighbor数量 
				Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = i;				//统计最大Common_Neighbor位置 
				Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;
			}
		}	
	} 
	if(Max_Common_Neighbor_num != 1)								//不止一个最大Common_Neighbor点
	{
#ifdef Debug
		cout << "Common_Neighbor的最大点不止一个" << endl; 
#endif
	 
		for(int k = Max_Common_Neighbor_num; k >= 1; k-- )
		{
			int i = Max_Common_Neighbor_i_Point[k]; 
			int j = Max_Common_Neighbor_j_Point[k];
#ifdef Debug
			cout << "i=" << i << endl; 
			cout << "j=" << j << endl; 
#endif
			if(Max_Degree < (Degree[i] + Degree[j]))				//找出度最大的两个点 
			{
				Max_Degree = Degree[i] + Degree[j];
				Max_Common_Neighbor_i_Point_Last = i;				//已经找出度最大的两个点 
				Max_Common_Neighbor_j_Point_Last = j;
			}
		}

	
		//Max_i = Max_Common_Neighbor_i_Point_Last						//已经找到最大Common_Neighbor点及最大度的edge(i,j) 
		//Max_j = Max_Common_Neighbor_j_Point_Last
#ifdef Debug
		cout << "最大度的Common_Neighbor的点"<<endl;
		cout << "Max_i=" << Max_Common_Neighbor_i_Point_Last << endl;
		cout << "Max_j=" << Max_Common_Neighbor_j_Point_Last << endl;
#endif
		Max_i =  Max_Common_Neighbor_i_Point_Last;
		Max_j =  Max_Common_Neighbor_j_Point_Last; 
	} 
	else															//仅有一个Common_Neighbor点
	{
#ifdef Debug
		cout << "Common_Neighbor的最大点仅为一个" << endl; 
		//Max_i = Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] //已经找到最大Common_Neighbor点及最大度的的edge(i,j) 
		//Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]
		cout << "Max_i=" << Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] << endl; 
		cout << "Max_j=" << Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] << endl; 
#endif
		Max_i = Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num];
		Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; 
	} 
	{ 
	//对Max_i 和Max_j 进行处理 就是对Clique分团进行处理，Node[],NodeList[][]都要进行处理
	//Madx_i 和Max_j 表示最大Common_Neighbor 并有最大度的两个连接点 edge(Max_i,Max_j)
	//Node[] == 1时表示这个点未受到处理，Node[] == 0时表示已经加入某个Clique
	//NodeList[i][1] == 1时表示这里有一个Clique NodeList[i][1] == 0时表示这里不存在Clique  
	
	//Clique[Max_i][Max_j].EdgeList 需要被处理 
	//存在于Clique[Max_i][]但不存在与Clique[Max_j][]中的点均为0 
	//Clique[Max_j][].EdgeList全部为0
	
	//Max_i  Max_j 目前仅对它们两个点进行操作 
	
	//对加入CLique进行处理 
	NodeList[Max_j][1] = 0;											//减少一个CLique
	Node[Max_j]		   = 0;											//减少一个Node
	
	//对边进行操作 删除仅与Max_i点相连的边 

	for(int j = 1; j <= n; j++)						//控制行的变化 
	{
		First[j] = Clique[Max_i][j].EdgeList;           //  提取第i行元素 
		Second[j] = Clique[Max_j][j].EdgeList;  //  提取第k行元素 
	}	
	for(int u = 1; u <= n; u++)
	{
		if(First[u] != Second[u] && First[u] == 1)			//存在一个点仅与相连的Max_i点相连 
		{
				Clique[Max_i][u].EdgeList = 0;
				Clique[u][Max_i].EdgeList = 0;
		}			
	}
	//将与Max_j相连的边进行删除 
	for(int i = 1; i <= n; i++)
	{
		Clique[Max_j][i].EdgeList = 0;	
		Clique[i][Max_j].EdgeList = 0;
	}
	
	
#ifdef Debug 	
	cout << "重新打印Clique[][].EdgeList矩阵" << endl;
	cout << "***************" << endl;
	cout << "print EdgeList " << endl; 
	for(int i = 1; i <= n; i++)
	{
		for(int j = 1; j <= n; j++)
		{
			cout << Clique[i][j].EdgeList;
			if(j == n)
				cout << endl;
		}
	}
	cout << "***************" <<endl;
#endif
	} 
#ifdef Debug 
	cout << "******************" << endl;
	for(int i = 1; i <= n; i++)
	{
		cout << "NodeList["<< i << "[1] = " << NodeList[i][1] << endl; 
	}
	cout << "结束max_Degree_Inceasing_Order = " <<max_Degree_Inceasing_Order << endl;
	//_sleep(5000);
#endif

}
//#ifdef Debug 
	/*计算Clique的个数*/
	cout << "******************" << endl;
	for(int i = 1; i <= n; i++)
	{
		if(NodeList[i][1] != 0)					//Clique保存在NodeList[][]中 
		{
			Clique_Count++;	
		} 
	} 
	cout << "print Clique_Count = " << Clique_Count << endl; 
//#endif
/*
以上已经构造完成无向图 G,下面在无向图G中应用最小团划分算法
首先找到节点度最大的两个顶点
当还有边存在的时候，就要继续进行 
*/
//Step1:
/*
Pick the edge(p,q) which has the maximum number of common neighbors(a
vertex is a common neighbor of an edge if it is connected with both
vertices of the edge).
-Tie-breaking: select p and q such that the sum of node degrees is
maximum;
-If the graph has no edges ,then Stop
*/
//统计度数
/*int Degree[n] = {0};
for(int i = 0; i < n; i++)
{
	for(int j = 0; j < n; j++)
	{
		Degree[i] += Clqiue.EdgeList[i][j];	
	}	
	cout << "Degree["<<i<<"]="<<Degree[i];
} 
cout << "throght Step1" << endl;
_sleep(1000);//delay 1s

Step2:*/
/*
Cluster p and q into a clique.
*/

//Step3:
/*
Delete edges from p and q that are not connected with their commom
neighbors.
*/
/*cout << "throght Step3"<<endl;
_sleep(1000);//delay 1s
Step4:*/
/*
Combine p and q in the original graph and call it r.
*/

//Step5:
/*
If vertex r is isolated ,Goto Step1
-Else pick an edge s which includes r as vertex and which has the 
maximum number of common neighbors.
*/
/*cout << "throght Step5" << endl;
_sleep(1000);//delay 1s
goto Step1;

Step6:*/
/*
Rename r and s as p and q.
*/

//Step7:
/*
Goto Step3.
*/
/*cout << "throght Step7"<<endl;
_sleep(1000);//delay 1s
goto Step3;

Step8: 
	
	cout << "Minimum_Clique_Partition_Algorithm"<<endl;
	return 0;*/	
	cout << "Do you want to continue? 【1】 Yes 【2】 No" << endl;
	int flag_go_on;
	cin >> flag_go_on;
	if(flag_go_on == 1)
	{
		goto go_on;
	}
	return 0;
}
