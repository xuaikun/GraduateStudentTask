﻿#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#define N 1004
#include <ctime>
#include <fstream>
using namespace std;

int SelectSort(int *A, int n);				//定义一个排序函数 

struct {									//感觉结构体比较适合~ 
	int EdgeList;							//无线图的边 
	int EdgeList_New;							//无线图的边 
	int Max_Common_Neighbor;				//数量最大的Common_Neighbor 
	int Common_Neighbor;					//计算每条边的Common_Neighbor数量 
	int Common_Neighbor_New;				//将Common_Neighbor数量统计到上三角矩阵中 
	int Edge_Delete;						//没用上  统计被删除的边 
	 
}Clique[N][N];						    	//二维矩阵结构体 

int main()
{
go_on:
	ifstream in;//定义读取文件流，相对于程序来说是in  为啥打开两次文件，第一次打开是为了知道 这个邻接矩阵的n为多少 
	in.open("what.txt");//打开文件
	
	srand((unsigned)time(0));
	int n;                                //定义节点个数 
	clock_t start, finish;
	int Clique_Count = 0;				  // 分团个数 
	                  
	string str;
	getline(in, str);
	cout << "length of str =" << str.length() << endl;
	
	n = (str.length()+1)/2;					 //输入节点个数  因为每两个数字之间会有一个空格，将空格的数量减掉 
	
	cout << "length of n =" << n << endl;	
	int Node[n];                  
	       //用于检验某个节点是否没有加入某个团 
	int NodeList[n][n];                 //检查节点都进入了哪个NodeList，记录生成了几个NodeList 
	int Node_New[n];
	int NodeList_New[n][n];
	
	int First[n];						//定义两个中间计算矩阵 与师姐一样~ 
	int Second[n];
	
	int Common_Neighbor_Sort[n];		//Common_Neighbor排序
	int Degree[n];						//统计每个节点的度 
	int Degree_Inceasing_Order[n];		//把度给排序
	 
	int max_Degree_Inceasing_Order = 1; //每次都需要 把每个节点度排序中的最大值拿出来 ，*********** 
										//将max_Degree_Inceasing_Order = 0作为while循环终止的条件 
										//max_Degree_Inceasing_Order = 0 意味着每个点都不在相连,可以结束查询 
	
	int Max_Degree = 0;					//当Common_Neighbor数量最大点不止两个时，需要找出当某前两个节点的最大度 
	int Min_Edge  = 10000;				//当Common_Neighbor数量最大点不止两个时，需要找出当前某两个节点删除的边的数量最少 
	for(int i = 1; i <= n; i++)          //初始化节点 
	{
		Node[i]        = 1;               //确保每个节点都存在  Node[i] == 1时表示存在此节点，否则表示不存在 
		NodeList[i][1] = i;               //确保初始化时每个节点存在于一个Clique中
		for(int j = 2; j <= n; j++)      //每个NodeList初始化时只有NodeList[i][1] = 1，j != 1时 NodeList[i][j] = 0; 
		{								//若i节点被加入到某个Clique中，那么 NodeList[i][1] = 0; 
			NodeList[i][j] = 0;			//i=1 to i->n 循环统计NodeList[i][1] 中等于 1的个数 就是Clique的个数 
		} 
 	}
	in.close();//读取完成之后关闭文件
	
	
	/*从这里开始赋值邻接矩阵*/ 
	ifstream infile;//定义读取文件流，相对于程序来说是in  	// 无向图从文件中读取邻接矩阵表示
	infile.open("ok.txt");//打开文件
	for (int i = 1; i <= n; i++)//定义行循环
	{
		for (int j = 1; j <= n; j++)//定义列循环
		{
			infile >> Clique[i][j].EdgeList;//读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
		}
	}
	infile.close();//读取完成之后关闭文件

	for(int i = 1; i <= n; i++)
	{
		for(int j = 1; j <= n; j++)
		{
			cout << Clique[i][j].EdgeList << " ";
			if(j == 5)
			{
				cout << endl;
			}
		}
	}

	for(int i = 1; i <= n; i++)							//把Clique[i][j].EdgeList的备份到Clique[i][j].EdgeList_New 
	{													//备份数据 
		Node_New[i] = Node[i];
		for(int j = 1; j <= n; j++)
		{
			Clique[i][j].EdgeList_New = Clique[i][j].EdgeList;
			NodeList_New[i][j]  = NodeList[i][j];
		}
	}
	start = clock();									//开始计时 
end1:
	
	while(max_Degree_Inceasing_Order)					// max_Degree_Inceasing_Order = 0 作为程序停止的标志，
														// max_Degree_Inceasing_Order 节点度排序后的最大值 最大值为0 表明每个节点不相连，
														//则全部点已经全部分到各个Clique 
	{
		
Step1_1:
		for(int i =1; i <= n; i++)
		{
			Common_Neighbor_Sort[i] = 0;	
		} 
	
		Max_Degree = 0;                                //为防止上次两个节点度的影响，最大度每次都得更新 
		 
		for(int i = 1; i <= n; i++)	                   //保证i < j   // 计算若选中这两个点后需要删除的边的数量 
		{
			for(int j = 1; j <= n; j++)
			{
				First[j] = Clique[i][j].EdgeList; 
			}
			for(int k = i + 1; k <= n; k++)
			{
				if(Clique[i][k].EdgeList == 1)					//保证这两个点是相连的，存在edge(i,j) 
				{
					int num = 0;								//需要删除的边 
					for(int u = 1; u <= n; u++)
					{
						Second[u] = Clique[k][u].EdgeList;
					}
					for(int q = 1; q <= n; q++)
					{
						if(First[q] != Second[q] && First[q] != 0)	//保证仅与较小点i相连，该删除 
						{
							num = num + 1; 							//时刻记得这样的情况
																	//0 0 1 
																	//0 0 0
																	//1 0 0 
						}
					}
					for(int c = 1; c <= n; c++)
					{
						if(Clique[k][c].EdgeList != 0)			//对于需要删除与j相连的边 
						{
							num = num + 1;
						}
					}
					Clique[i][k].Edge_Delete = num - 1;							//为什么要减1？ 本身为1，但对方对应不为1，多加了1条边，所以减1 
					Clique[k][i].Edge_Delete = Clique[i][k].Edge_Delete;		//其实是同一条边，删除的边是一样的 
				}
				else{
					Clique[i][k].Edge_Delete = 0;
				} 
			}
		} 		
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

		if(max_Degree_Inceasing_Order == 0)						//以获取最大节点度为0，马上终止程序 
		{
			goto end1;
		} 
	
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
					//int num = 0;								//需要删除的边
					for(int u = 1; u <= n; u++)
					{
						if(First[u] == Second[u] && First[u] != 0)			//相连的两个点(edge) 有共同的点（Common_Neignbor）,并且0不算相连 
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
			for(int j = 1; j <= n; j++)
			{	
				if(i <= j)
				{
					Clique[i][j].Common_Neighbor_New = Clique[i][j].Common_Neighbor; //将Common_Neighbor的上三角提取出来 
				} 
				if(j == n)
				{
				} 	
			}	
		} 

		/*******************************************************/
		/***********Common_Neighbor_Sort出了问题****************/
		/*******************************************************/
		/*将每一个节点对应的最大Common_Neighbor找出来*/
		for(int i = 1, j = 1; i <= n; i++)
		{														//*********************之前这里好像错了***************************// 
			int max = Clique[i][i].Common_Neighbor_New;         //从对角线开始  ~~~~~~~~~~~~才是正确的  这个值等于0  不然会出现随机值 
			for(j = i; j <= n; j++)
			{
				if(max < Clique[i][j].Common_Neighbor_New)
				{ 
					max = Clique[i][j].Common_Neighbor_New;	//若当前max值比 Clique.Common_Neighbor_New[i][j]小，则替换 max值 
				}
			}
			Common_Neighbor_Sort[i] = max;
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
		int Max_j;	
		Max_Common_Neighbor_num = 0;										//一定存在 Max_j > Max_i 
		
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
		Max_Degree = 0;
		Min_Edge = 10000;
		if(Max_Common_Neighbor_num != 1)								//含有最大Common_Neighbor的边不止一条 
		{
			for(int k = Max_Common_Neighbor_num; k >= 1; k-- )		  //通过Max_Common_Neighbor_num作为index将它们一条条找出来 
			{														  //并把选中边中度最大的找出来 
				int i = Max_Common_Neighbor_i_Point[k]; 
				int j = Max_Common_Neighbor_j_Point[k];
			/*找出需要删除的边最少的两个点*/
			//Clique[i][j].Edge_Delete
				if(Min_Edge > Clique[i][j].Edge_Delete)
				{
					Min_Edge = Clique[i][j].Edge_Delete;
					Max_Common_Neighbor_i_Point_Last = i;
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
		} //end Step1
Step2_1:
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
		int temp;
		if(Max_i > Max_j)										//若Max_i较大，则互换Max_i与Max_j的值 
		{
			temp  = Max_j;
			Max_j = Max_i;
			Max_i = temp; 
		}
				
		NodeList[Max_j][1] = 0;							     //减少一个CLique
		Node[Max_j]		   = 0;					  	      	//减少一个Node 均表示将Max_j与Max_i合并为Max_i 
		
															//保存同在一个团里面  考虑一个问题~  
															//就是当原来的Max_i > Max_j 时要把 Max_i 与 Max_j互换
															//NodeList[Max_j][i]的值也得保存的要保存到NodeList[Max_i][i]  
		{
			NodeList[Max_i][Max_j] = Max_j;					//主要是NodeList[Max_j][1] = Max_j 
			for(int i = 2; i <= n; i++)
			{
				if(NodeList[Max_j][i] != 0)					//将属于NodeList[Max_j][i]归并到NodeList[Max_i][i]中 
				{
					NodeList[Max_i][i] = i;
				}	
			}
		}		
		for(int i = 1; i <= n; i++)								//NodeList[Max_j][i]中的点已经被归并到NodeList[Max_i][i]了，将Nodelist[Max_j][i]清零 
		{
			NodeList[Max_j][i] = 0;	
		}											

Step3_1:
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
		// 将与标号大的点即Max_j相连的边全部删掉	//这样做少了一些工作，比较好理解，可能直接说不通 
		for(int i = 1; i <= n; i++)
		{
			Clique[Max_j][i].EdgeList = 0;			//生成的是无向图，需要置对称的两个值为0，即为删除边 
			Clique[i][Max_j].EdgeList = 0;
		}
		
Step4_1:
				Max_i;									 	//Max_i 和 Max_j 合并为 Max_i 
		} 
Step5_1:
		/*由于节点被修改，每个节点的度也被修改了 重新计算度的值*/ 
			for(int i = 1; i <= n; i++)
			{
				Degree[i] = 0;										//每个节点的度初始化为0 
				for(int j = 1; j <= n; j++)
				{
					Degree[i] = Degree[i] + Clique[i][j].EdgeList; // 每一行的值的总和即为某个节点的度 
				}	
				Degree_Inceasing_Order[i] = Degree[i];				//将Degree[]的值依次存入Degree_Increasing_Order[]用于排序
																	//依旧要保存Degree不变，方便 “查询时节点” 时使用 
			} 										//将节点的度进行排序~ 
			SelectSort(Degree_Inceasing_Order, n);
			
			max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];

			for(int i = 1; i <= n; i++)	//保证i < j   // 计算若选中这两个点后需要删除的边的数量 
			{
				for(int j = 1; j <= n; j++)
				{
					First[j] = Clique[i][j].EdgeList; 
				}
				for(int k = i + 1; k <= n; k++)
				{
					if(Clique[i][k].EdgeList == 1)					//保证这两个点是相连的，存在edge(i,j) 
					{
						int num = 0;								//需要删除的边 
						for(int u = 1; u <= n; u++)
						{
							Second[u] = Clique[k][u].EdgeList;
						}
						for(int q = 1; q <= n; q++)
						{
							if(First[q] != Second[q] && First[q] != 0)	//保证仅与较小点i相连，该删除 
							{
								num = num + 1; 
							}
						}
						for(int c = 1; c <= n; c++)
						{
							if(Clique[k][c].EdgeList != 0)			//对于需要删除与j相连的边 
							{
								num = num + 1;
							}
						}
						Clique[i][k].Edge_Delete = num - 1; 
						Clique[k][i].Edge_Delete = Clique[i][k].Edge_Delete; //其实是同一条边，删除的边是一样的  
					}
					else{
						Clique[i][k].Edge_Delete = 0;
					} 
				}
			} 
			if(max_Degree_Inceasing_Order == 0)						//以获取最大节点度为0，马上终止程序  Max_i 已经单独存在 
			{
				goto end1;
			}  	

			if(Degree[Max_i] == 0)					//如果Max_i节点独立了，另选一条边即另外两个点 
			{
				goto Step1_1;
			}
			else								    // 还有与Max_i相连的边 
			{
				/*计算包含Max_i节点的边的Common_Neighbor的最大值情况*/	
				for(int j = 1; j <= n; j++)						//控制行的变化 
				{
					First[j] = Clique[Max_i][j].EdgeList;           //  提取第i行元素 
				}	
				for(int k = 1; k <= n; k++)						//控制列的变化
				{
					if(Clique[Max_i][k].EdgeList == 1)				//保证操作的两个点是相连的 
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
						Clique[Max_i][k].Common_Neighbor = Diff;    //每条边的Common_Neighbor数量 
					}
					else
						Clique[Max_i][k].Common_Neighbor = 0;	
				}
				//	只获取Common_Neighbor上三角部分数据，保证Common_Neighbor[][]只保存不同边的个数 
				for(int j = 1; j <= n; j++)						//不知道Max_i 和 j谁大谁小 
				{
					Clique[Max_i][j].Common_Neighbor_New = Clique[Max_i][j].Common_Neighbor; //将Common_Neighbor的上三角提取出来 
				}	
	
				//已经获得 Clique[Max_i][k].Common_Neighbor_New	
				int max = Clique[Max_i][1].Common_Neighbor_New; 
				for(int j = 2; j <= n; j++)
				{
					if(max < Clique[Max_i][j].Common_Neighbor_New)
					{ 
						max = Clique[Max_i][j].Common_Neighbor_New;	//若当前max值比 Clique.Common_Neighbor_New[i][j]小，则替换 max值 
					}
				}
				Common_Neighbor_Sort[1] = max;
				//Common_Neighbor_Sort[Max_i] = max;					// Max_i最大的Common_Neighbor 为 max 
				//最大的Common_Neighbor = max
				//两个顶点分别为 Max_i Max_j 判断max 是否唯一
				Max_Common_Neighbor_num = 0;
				for(int j = 1; j <= n; j++)
				{
					// Common_Neighbor数量最大点为 Common_Neighbor_Sort[1]  每次都只拿最大的出来比较即可  
					if(Common_Neighbor_Sort[1] == Clique[Max_i][j].Common_Neighbor_New && Clique[Max_i][j].EdgeList != 0)	
					//找到最大Common_Neighbor的两个节点 防止最大Common_Neighbor为0时出现错误，保证选中的两个节点必须相连 
					{
						Max_Common_Neighbor_num++;					 						//统计最大Common_Neighbor数量 
						//Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = Max_i;			//统计最大Common_Neighbor位置 即：两个节点标号 
						Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;			//Max_Common_Neighbor_num可以作为index 
					}
				}	
				Max_Degree = 0;
				Min_Edge   = 10000;
				if(Max_Common_Neighbor_num != 1)								//含有最大Common_Neighbor的边不止一条 
				{
					for(int k = Max_Common_Neighbor_num; k >= 1; k-- )		  //通过Max_Common_Neighbor_num作为index将它们一条条找出来 
					{														  //并把选中边中度最大的找出来 
						 
						int j = Max_Common_Neighbor_j_Point[k];
						if(Min_Edge > Clique[Max_i][j].Edge_Delete)                //找出删除边最少边的两个点 
						{
							Min_Edge = Clique[Max_i][j].Edge_Delete;
							Max_Common_Neighbor_j_Point_Last = j;
						}
						
					}
																					//已经找到最大Common_Neighbor点及最大度的edge(i,j) 
					Max_j =  Max_Common_Neighbor_j_Point_Last; 
				} 
				else															//最大Common_Neighbor的边只有一条 
				{
																				 //已经找到最大Common_Neighbor点及最大度的的edge(i,j) 
					Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; //Max_Common_Neighbor_num就是index 
				} 
				{
					//重新获得 Max_i 和 Max_j 两个节点为最有选择点 
					//Max_i;
					//Max_j;
					goto Step2_1; 
				} 
			}			
	}//end while(Max_Degree_Increasing_Order)
	
	/*计算Clique的个数*/
	cout << "******************" << endl;
	for(int i = 1; i <= n; i++)
	{
		if(NodeList[i][1] != 0)					//Clique保存在NodeList[][]中 NodeList[i][1] = 1表示这里有一个团
												//NodeList[i][1] = 0表示这里没有团 
		{
			Clique_Count++;	
		} 
	} 
	cout << "print Clique_Count = " << Clique_Count << endl; 
	for(int i = 1; i <= n; i++)
	{
		if(NodeList[i][1] != 0)									//保证这点的根节点一定存在~ 
		{
			cout << "Clique[" << i << "]:( " << i << ",";		//在检查它其中还包含哪些节点 
			for(int j = i + 1; j <= n; j++)
			{
				if(NodeList[i][j] != 0)
				{
					cout << j << ",";
				}	
			}
			cout << ")" << endl;
		}

	}
	finish = clock();
	
	cout <<"**********************" << endl;
	cout << "Edge_Delete Time = " << (finish - start )/CLOCKS_PER_SEC << "s" << endl;
	cout <<"**********************" << endl;
	/***********************************************************************/
	
	
	for(int i = 1; i <= n; i++)							//把Clique[i][j].EdgeList的备份到Clique[i][j].EdgeList_New 
	{													//备份数据 
		Node[i] = Node_New[i];
		for(int j = 1; j <= n; j++)
		{
			Clique[i][j].EdgeList = Clique[i][j].EdgeList_New;
			NodeList[i][j]  = NodeList_New[i][j];
		}
	}
	max_Degree_Inceasing_Order = 1;					//重新来一次 
	Clique_Count = 0;
	
	start = clock();
end2:
	
	while(max_Degree_Inceasing_Order)					// max_Degree_Inceasing_Order = 0 作为程序停止的标志，
														// max_Degree_Inceasing_Order 节点度排序后的最大值 最大值为0 表明每个节点不相连，
														//则全部点已经全部分到各个Clique 
	{
		
Step1_2:
		for(int i =1; i <= n; i++)						//每次都把 最大Common_Neighbor排序数组置零 
		{
			Common_Neighbor_Sort[i] = 0;	
		} 
	
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
									
		SelectSort(Degree_Inceasing_Order, n);					//将节点的度进行排序~ 
		
		max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];

		if(max_Degree_Inceasing_Order == 0)						//以获取最大节点度为0，马上终止程序 
		{
			goto end2;
		} 
	
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
						if(First[u] == Second[u] && First[u] != 0)			//相连的两个点(edge) 有共同的点（Common_Neignbor）,并且0不算相连 
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
			for(int j = 1; j <= n; j++)
			{		
				if(i <= j)
				{
					Clique[i][j].Common_Neighbor_New = Clique[i][j].Common_Neighbor; //将Common_Neighbor的上三角提取出来 
				} 
				if(j == n)
				{	
				} 	
			}	
		} 

		/*******************************************************/
		/***********Common_Neighbor_Sort出了问题****************/
		/*******************************************************/
		/*将每一个节点对应的最大Common_Neighbor找出来*/
		for(int i = 1, j = 1; i <= n; i++)
		{														//*********************之前这里好像错了***************************// 
			int max = Clique[i][i].Common_Neighbor_New;         //从对角线开始  ~~~~~~~~~~~~才是正确的  这个值等于0  不然会出现随机值 
			for(j = i; j <= n; j++)
			{
				if(max < Clique[i][j].Common_Neighbor_New)
				{ 
					max = Clique[i][j].Common_Neighbor_New;	//若当前max值比 Clique.Common_Neighbor_New[i][j]小，则替换 max值 
				}
			}
			Common_Neighbor_Sort[i] = max;
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
		int Max_j;	
		Max_Common_Neighbor_num = 0;										//一定存在 Max_j > Max_i 
		
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

		Max_Degree = 0;
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
		} //end Step1
Step2_2:
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
	
			int temp;
			if(Max_i > Max_j)										//若Max_i较大，则互换Max_i与Max_j的值 
			{
				temp  = Max_j;
				Max_j = Max_i;
				Max_i = temp; 
			}
					
			NodeList[Max_j][1] = 0;							     //减少一个CLique
			Node[Max_j]		   = 0;					  	      	//减少一个Node 均表示将Max_j与Max_i合并为Max_i 
			
																//保存同在一个团里面  考虑一个问题~  
																//就是当原来的Max_i > Max_j 时要把 Max_i 与 Max_j互换
																//NodeList[Max_j][i]的值也得保存的要保存到NodeList[Max_i][i]  
			{
				NodeList[Max_i][Max_j] = Max_j;					//主要是NodeList[Max_j][1] = Max_j 
				for(int i = 2; i <= n; i++)
				{
					if(NodeList[Max_j][i] != 0)					//将属于NodeList[Max_j][i]归并到NodeList[Max_i][i]中 
					{
						NodeList[Max_i][i] = i;
					}	
				}
			}		
			for(int i = 1; i <= n; i++)								//NodeList[Max_j][i]中的点已经被归并到NodeList[Max_i][i]了，将Nodelist[Max_j][i]清零 
			{
				NodeList[Max_j][i] = 0;	
			}											
	
Step3_2:
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
			
Step4_2:
			Max_i;									 	//Max_i 和 Max_j 合并为 Max_i 
		} 
	
Step5_2:
		/*由于节点被修改，每个节点的度也被修改了 重新计算度的值*/ 
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
			/*打印每个节点的度*/
										//将节点的度进行排序~ 
			SelectSort(Degree_Inceasing_Order, n);
			
			max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];

			if(max_Degree_Inceasing_Order == 0)						//以获取最大节点度为0，马上终止程序 
			{
				goto end2;
			}  	

			if(Degree[Max_i] == 0)					//如果Max_i节点独立了，另选一条边即另外两个点 
			{
				goto Step1_2;
			}
			else								    // 还有与Max_i相连的边 
			{
				/*计算包含Max_i节点的边的Common_Neighbor的最大值情况*/	
				for(int j = 1; j <= n; j++)						//控制行的变化 
				{
					First[j] = Clique[Max_i][j].EdgeList;           //  提取第i行元素 
				}	
				for(int k = 1; k <= n; k++)						//控制列的变化
				{
					if(Clique[Max_i][k].EdgeList == 1)				//保证操作的两个点是相连的 
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
						Clique[Max_i][k].Common_Neighbor = Diff;    //每条边的Common_Neighbor数量 
					}
					else
						Clique[Max_i][k].Common_Neighbor = 0;	
				}
				//	只获取Common_Neighbor上三角部分数据，保证Common_Neighbor[][]只保存不同边的个数 
				for(int j = 1; j <= n; j++)						//不知道Max_i 和 j谁大谁小 
				{
					Clique[Max_i][j].Common_Neighbor_New = Clique[Max_i][j].Common_Neighbor; //将Common_Neighbor的上三角提取出来 
				}	
	
				//已经获得 Clique[Max_i][k].Common_Neighbor_New	
				int max = Clique[Max_i][1].Common_Neighbor_New; 
				for(int j = 2; j <= n; j++)
				{
					if(max < Clique[Max_i][j].Common_Neighbor_New)
					{ 
						max = Clique[Max_i][j].Common_Neighbor_New;	//若当前max值比 Clique.Common_Neighbor_New[i][j]小，则替换 max值 
					}
				}
				Common_Neighbor_Sort[1] = max;
				// Max_i最大的Common_Neighbor 为 max 
				//最大的Common_Neighbor = max
				//两个顶点分别为 Max_i Max_j 判断max 是否唯一
				Max_Common_Neighbor_num = 0;
				for(int j = 1; j <= n; j++)
				{
					// Common_Neighbor数量最大点为 Common_Neighbor_Sort[1]  每次都只拿最大的出来比较即可  
					if(Common_Neighbor_Sort[1] == Clique[Max_i][j].Common_Neighbor_New && Clique[Max_i][j].EdgeList != 0)	
					//找到最大Common_Neighbor的两个节点 防止最大Common_Neighbor为0时出现错误，保证选中的两个节点必须相连 
					{
						Max_Common_Neighbor_num++;					 						//统计最大Common_Neighbor数量 
						//Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = Max_i;			//统计最大Common_Neighbor位置 即：两个节点标号 
						Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;			//Max_Common_Neighbor_num可以作为index 
					}
				}	
				Max_Degree = 0;
				if(Max_Common_Neighbor_num != 1)								//含有最大Common_Neighbor的边不止一条 
				{
					for(int k = Max_Common_Neighbor_num; k >= 1; k-- )		  //通过Max_Common_Neighbor_num作为index将它们一条条找出来 
					{														  //并把选中边中度最大的找出来 
						 
						int j = Max_Common_Neighbor_j_Point[k];
						if(Max_Degree < (Degree[Max_i] + Degree[j]))				//找出度最大的两个点 
						{
							Max_Degree = Degree[Max_i] + Degree[j];
																					//已经找出度最大的两个点 
							Max_Common_Neighbor_j_Point_Last = j;
						}
					}
																					//已经找到最大Common_Neighbor点及最大度的edge(i,j) 
					Max_j =  Max_Common_Neighbor_j_Point_Last; 
				} 
				else															//最大Common_Neighbor的边只有一条 
				{
																				 //已经找到最大Common_Neighbor点及最大度的的edge(i,j) 
					Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; //Max_Common_Neighbor_num就是index 
				} 
				{
					//重新获得 Max_i 和 Max_j 两个节点为最有选择点 
					//Max_i;
					//Max_j;
					goto Step2_2; 
				} 
			}			
	}//end while(Max_Degree_Increasing_Order)
	
	//#ifdef Debug 
		/*计算Clique的个数*/
		cout << "******************" << endl;
		for(int i = 1; i <= n; i++)
		{
			if(NodeList[i][1] != 0)					//Clique保存在NodeList[][]中 NodeList[i][1] = 1表示这里有一个团
													//NodeList[i][1] = 0表示这里没有团 
			{
				Clique_Count++;	
			} 
		} 
		cout << "print Clique_Count = " << Clique_Count << endl; 
		for(int i = 1; i <= n; i++)
		{
			if(NodeList[i][1] != 0)									//保证这点的根节点一定存在~ 
			{
				cout << "Clique[" << i << "]:( " << i << ",";		//在检查它其中还包含哪些节点 
				for(int j = i + 1; j <= n; j++)
				{
					if(NodeList[i][j] != 0)
					{
						cout << j << ",";
					}	
				}
				cout << ")" << endl;
			}
		}
	
	finish = clock();
	
	cout <<"**********************" << endl;
	cout << "Degree_Delete Time = " << (finish - start )/CLOCKS_PER_SEC<< "s" << endl;
	cout <<"**********************" << endl; 
	//#endif 

	cout << "Do you want to continue? 【1】 Yes 【2】 No" << endl;
	int flag_go_on;
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
