#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#define N 1004
using namespace std;

int SelectSort(int *A, int n);				//定义一个排序函数 

struct {								   //感觉结构体比较适合~ 
	int EdgeList;						   //无线图的边 
	int Max_Common_Neighbor;			   //数量最大的Common_Neighbor 
	int Common_Neighbor;				   //计算每条边的Common_Neighbor数量 
	int Common_Neighbor_New;			   //将Common_Neighbor数量统计到上三角矩阵中 
	int Edge_Delete;					   //没用上  统计被删除的边 
	 
}Clique[N][N];							   //二维矩阵结构体 

int main()
{
go_on://跳转标志 
	srand((unsigned)time(0));
	int n;                                //定义节点个数 
	int Clique_Count = 0;				  // 分团个数 
	cout << "输入节点个数：";
	cin >> n;                            //输入节点个数 
	int Node[n];                         //用于检验某个节点是否没有加入某个团 
	int NodeList[n][n];                  //检查节点都进入了哪个NodeList，记录生成了几个NodeList 
	int First[n];					     //定义两个中间计算矩阵 与师姐一样~ 
	int Second[n];
	int Common_Neighbor_Sort[n];	  	//Common_Neighbor排序
	int Degree[n];						//统计每个节点的度 
	int Degree_Inceasing_Order[n];		//把度给排序
	int max_Degree_Inceasing_Order = 1; //每次都需要 把每个节点度排序中的最大值拿出来 ，*********** 
										//将max_Degree_Inceasing_Order = 0作为while循环终止的条件 
										//max_Degree_Inceasing_Order = 0 意味着每个点都不在相连,可以结束查询 
	int Max_Degree = 0;					//当Common_Neighbor数量最大点不止两个时，需要找出当某前两个节点的最大度 
	int flag_go_on;						//继续运行的标志 
	for(int i = 1; i <= n; i++)          //初始化节点 
	{
		Node[i]        = 1;               //确保每个节点都存在  Node[i] == 1时表示存在此节点，否则表示不存在 
		NodeList[i][1] = 1;               //确保初始化时每个节点存在于一个Clique中
		for(int j = 2; j <= n; j++)      //每个NodeList初始化时只有NodeList[i][1] = 1，j != 1时 NodeList[i][j] = 0; 
		{								//若i节点被加入到某个Clique中，那么 NodeList[i][1] = 0; 
			NodeList[i][j] = 0;			//i=1 to i->n 循环统计NodeList[i][1] 中等于 1的个数 就是Clique的个数 
		} 
 	}
 	
	for(int i = 1; i <= n; i++)			// 无向图用随机生成的邻接矩阵表示 
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

	while(max_Degree_Inceasing_Order)					// max_Degree_Inceasing_Order = 0 作为程序停止的标志，
														// max_Degree_Inceasing_Order 节点度排序后的最大值 最大值为0 表明每个节点不相连，
														//可独立为Clique 
	{
		Max_Degree = 0;                                //为防止上次两个节点度的影响，最大度每次都得更新 
										    			// 统计每个节点的度保存到Degree[],数组下表即为节点的标号									
		for(int i = 1; i <= n; i++)
		{
			Degree[i] = 0;										//每个节点的度初始化为0 
			for(int j = 1; j <= n; j++)
			{
				Degree[i] = Degree[i] + Clique[i][j].EdgeList; // 每一行的值的总和即为某个节点的度 
			}	
			Degree_Inceasing_Order[i] = Degree[i];				//将Degree[]的值依次存入Degree_Increasing_Order[]用于排序
																//依旧要保存Degree不变，方便 “查询时节点” 时使用 
		} 
																//将节点的度进行排序~ 
		SelectSort(Degree_Inceasing_Order, n);
		max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];
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
		for(int i = 1; i <= n; i++)				//	只获取Common_Neighbor上三角部分数据，保证Common_Neighbor[][]只保存不同边的个数 
		{
			for(int j = i; j <= n; j++)
			{
				Clique[i][j].Common_Neighbor_New = Clique[i][j].Common_Neighbor; //将Common_Neighbor的上三角提取出来 
			}	
		} 
		
		/*将每一个节点对应的最大Common_Neighbor找出来*/
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
		
		SelectSort(Common_Neighbor_Sort, n);			//对统计出来的每个点的最大Common_Neighbor数量进行排序 
		
		/*首先找出Common_Neighbor最大的，如Common_Neighbor最大值唯一，则继续，如最大值不唯一则再比较
		Degree[],有限合并度最大的两个节点*/
		int Max_Common_Neighbor_num = 0; 					//统计有用最大Common_Neighbor的边的数量 
		int Max_Common_Neighbor_i_Point[n];					//保存每条边的i节点 
		int Max_Common_Neighbor_j_Point[n];					//保存每条边的j节点 
		int Max_Common_Neighbor_i_Point_Last;				//当最大Common_Neighbor的边不止一条时，需要找度最大的那两个节点 获取i节点 
		int Max_Common_Neighbor_j_Point_Last;				//获取j节点 
		int Max_i;											//最优的两个点~的 标号 
		int Max_j;											//一定存在 Max_j > Max_i 
		for(int i = 1; i <= n; i++)
		{
			for(int j = i + 1; j <= n; j++)
			{
				// Common_Neighbor数量最大点为 Common_Neighbor_Sort[1]  每次都只拿最大的出来比较即可  
				if(Common_Neighbor_Sort[1] == Clique[i][j].Common_Neighbor_New && Clique[i][j].EdgeList != 0)	
				//找到最大Common_Neighbor的两个节点 防止最大Common_Neighbor为0时出现错误，保证选中的两个节点必须相连 
				{
					Max_Common_Neighbor_num++;					 						//统计最大Common_Neighbor数量 
					Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = i;			//统计最大Common_Neighbor位置 即：两个节点标号 
					Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;			//Max_Common_Neighbor_num可以作为index 
				}
			}	
		} 
		
		if(Max_Common_Neighbor_num != 1)								//含有最大Common_Neighbor的边不止一条 
		{
			for(int k = Max_Common_Neighbor_num; k >= 1; k-- )		  //通过Max_Common_Neighbor_num作为index将它们一条条找出来 
			{														  //并把选中边中度最大的找出来 
				int i = Max_Common_Neighbor_i_Point[k]; 
				int j = Max_Common_Neighbor_j_Point[k];
				if(Max_Degree < (Degree[i] + Degree[j]))				//找出度最大的两个点 
				{
					Max_Degree = Degree[i] + Degree[j];
					Max_Common_Neighbor_i_Point_Last = i;				//已经找出度最大的两个点 
					Max_Common_Neighbor_j_Point_Last = j;
				}
			}
			Max_i =  Max_Common_Neighbor_i_Point_Last;					//已经找到最大Common_Neighbor点及最大度的edge(i,j) 
			Max_j =  Max_Common_Neighbor_j_Point_Last; 
		} 
		else															//最大Common_Neighbor的边只有一条 
		{
			Max_i = Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num]; //已经找到最大Common_Neighbor点及最大度的的edge(i,j) 
			Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; //Max_Common_Neighbor_num就是index 
		} 
		{ 
			//对Max_i 和Max_j 进行处理 就是对Clique分团进行处理，Node[],NodeList[][]都要进行处理
			//Madx_i 和Max_j 表示最大Common_Neighbor 并有最大度的两个连接点 edge(Max_i,Max_j)
			//Node[] == 1时表示这个点未受到处理，Node[] == 0时表示已经加入某个Clique
			//NodeList[i][1] == 1时表示这里有一个Clique NodeList[i][1] == 0时表示这里不存在Clique  
			
			//Clique[Max_i][Max_j].EdgeList 需要被处理 
			//存在于Clique[Max_i][]但不存在与Clique[Max_j][]中的点均为0 
			//Clique[Max_j][].EdgeList全部为0
			
			//已经得到 Max_i  Max_j 目前仅对它们两个点进行操作  ************************ 
			 
			//对加入CLique进行处理  一定存在 Max_i < Max_j  
			NodeList[Max_j][1] = 0;							     //减少一个CLique
			Node[Max_j]		   = 0;					  	      	//减少一个Node 均表示将Max_j与Max_i合并为Max_i 
		
			//对边进行操作 删除仅与Max_i点相连的边 (师姐解释的，第三个点仅与其中一个点相连，它们的边删除)
			for(int j = 1; j <= n; j++)					     	//控制行的变化 
			{
				First[j] = Clique[Max_i][j].EdgeList;         //  提取第i行元素 
				Second[j] = Clique[Max_j][j].EdgeList;       //  提取第k行元素 
			}	
			for(int u = 1; u <= n; u++)
			{												//已经明确Max_i和Max_j是相连的，查看它们对应邻接矩阵的Max_i行和Max_j行
															//对应列是否相等，相等且等于1表示第三个点k与Max_i、Max_j相连
															//则Edge(Max_i,k)(如果k>Max_i时 则为Edge(k, Max_i))不需要删除，反之需要删除 
				if(First[u] != Second[u] && First[u] == 1)	//存在一个点仅与相连的Max_i点相连 即 Clique[Max_i][u].EdgeList = 1;
				{
						Clique[Max_i][u].EdgeList = 0;		//生成的是无向图，需要置对称的两个值为0，即为删除边 
						Clique[u][Max_i].EdgeList = 0;
				}			
			}
			// 将与标号大的点即Max_j相连的边全部删掉	//这样做少了一些工作，比较好理解，可以直接说不通 
			for(int i = 1; i <= n; i++)
			{
				Clique[Max_j][i].EdgeList = 0;			//生成的是无向图，需要置对称的两个值为0，即为删除边 
				Clique[i][Max_j].EdgeList = 0;
			}
		} 
	} //end while(max_Degree_Inceasing_Order) 		
											
	cout << "******************" << endl;
	for(int i = 1; i <= n; i++)					/*统计Clique的个数*/
	{
		if(NodeList[i][1] != 0)					//Clique保存在NodeList[][]中 NodeList[i][1] = 1表示这里有一个团
												//NodeList[i][1] = 0表示这里没有团 
		{
			Clique_Count++;	
		} 
	} 
	cout << "print Clique_Count = " << Clique_Count << endl; 
	cout << "Do you want to continue? 【1】 Yes 【2】 No" << endl;
	cin >> flag_go_on;
	if(flag_go_on == 1)
	{
		goto go_on;
	}
	return 0;
}

int SelectSort(int *A, int n)				//定义一个排序函数
{
	int temp;
	for(int i = 1; i < n ; i++)
	{
		int max = i;
		for(int j = i + 1; j <= n; j++)
		{
			if(A[max] < A[j])
			{
				max = j;
			}		
		}
		{
			temp = A[max];
			A[max] = A[i];
			A[i]   = temp;
		}
	} 
	return 0;
}
