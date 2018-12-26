#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
//#define Debug                              //�����õ�  
#define N 1004
using namespace std;

int SelectSort(int *A, int n);				//����һ�������� 

struct {									//�о��ṹ��Ƚ��ʺ�~ 
	int EdgeList;							//����ͼ�ı� 
	int Max_Common_Neighbor;				//��������Common_Neighbor 
	int Common_Neighbor;					//����ÿ���ߵ�Common_Neighbor���� 
	int Common_Neighbor_New;				//��Common_Neighbor����ͳ�Ƶ������Ǿ����� 
	int Edge_Delete;						//û����  ͳ�Ʊ�ɾ���ı� 
	 
}Clique[N][N];						    	//��ά����ṹ�� 

int main()
{
go_on:
	srand((unsigned)time(0));
	int n;                                //����ڵ���� 
	int Clique_Count = 0;				  // ���Ÿ��� 
	cout << "����ڵ������";
	cin >> n;                            //����ڵ���� 
	int Node[n];                         //���ڼ���ĳ���ڵ��Ƿ�û�м���ĳ���� 
	int NodeList[n][n];                 //���ڵ㶼�������ĸ�NodeList����¼�����˼���NodeList 
	
	int First[n];						//���������м������� ��ʦ��һ��~ 
	int Second[n];
	
	int Common_Neighbor_Sort[n];		//Common_Neighbor����
	int Degree[n];						//ͳ��ÿ���ڵ�Ķ� 
	int Degree_Inceasing_Order[n];		//�Ѷȸ�����
	 
	int max_Degree_Inceasing_Order = 1; //ÿ�ζ���Ҫ ��ÿ���ڵ�������е����ֵ�ó��� ��*********** 
										//��max_Degree_Inceasing_Order = 0��Ϊwhileѭ����ֹ������ 
										//max_Degree_Inceasing_Order = 0 ��ζ��ÿ���㶼��������,���Խ�����ѯ 
	
	int Max_Degree = 0;					//��Common_Neighbor�������㲻ֹ����ʱ����Ҫ�ҳ���ĳǰ�����ڵ������ 
	
	for(int i = 1; i <= n; i++)          //��ʼ���ڵ� 
	{
		Node[i]        = 1;               //ȷ��ÿ���ڵ㶼����  Node[i] == 1ʱ��ʾ���ڴ˽ڵ㣬�����ʾ������ 
		NodeList[i][1] = i;               //ȷ����ʼ��ʱÿ���ڵ������һ��Clique��
		for(int j = 2; j <= n; j++)      //ÿ��NodeList��ʼ��ʱֻ��NodeList[i][1] = 1��j != 1ʱ NodeList[i][j] = 0; 
		{								//��i�ڵ㱻���뵽ĳ��Clique�У���ô NodeList[i][1] = 0; 
			NodeList[i][j] = 0;			//i=1 to i->n ѭ��ͳ��NodeList[i][1] �е��� 1�ĸ��� ����Clique�ĸ��� 
		} 
 	}
	
	for(int i = 1; i <= n; i++)			// ����ͼ��������ɵ��ڽӾ����ʾ 
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

end:
	
	while(max_Degree_Inceasing_Order)					// max_Degree_Inceasing_Order = 0 ��Ϊ����ֹͣ�ı�־��
														// max_Degree_Inceasing_Order �ڵ�����������ֵ ���ֵΪ0 ����ÿ���ڵ㲻������
														//��ȫ�����Ѿ�ȫ���ֵ�����Clique 
	{
		
Step1:
		for(int i =1; i <= n; i++)						//ÿ�ζ��� ���Common_Neighbor������������ 
		{
			Common_Neighbor_Sort[i] = 0;	
		} 
	
		Max_Degree = 0;                                //Ϊ��ֹ�ϴ������ڵ�ȵ�Ӱ�죬����ÿ�ζ��ø��� 
		 
																// ͳ��ÿ���ڵ�Ķȱ��浽Degree[],�����±�Ϊ�ڵ�ı��									
		for(int i = 1; i <= n; i++)
		{
			Degree[i] = 0;										//ÿ���ڵ�Ķȳ�ʼ��Ϊ0 
			for(int j = 1; j <= n; j++)
			{
				Degree[i] = Degree[i] + Clique[i][j].EdgeList; // ÿһ�е�ֵ���ܺͼ�Ϊĳ���ڵ�Ķ� 
			}	
			Degree_Inceasing_Order[i] = Degree[i];				//��Degree[]��ֵ���δ���Degree_Increasing_Order[]��������
																//����Ҫ����Degree���䣬���� ����ѯʱ�ڵ㡱 ʱʹ�� 
		} 
									
		SelectSort(Degree_Inceasing_Order, n);					//���ڵ�ĶȽ�������~ 
		
		max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];

		if(max_Degree_Inceasing_Order == 0)						//�Ի�ȡ���ڵ��Ϊ0��������ֹ���� 
		{
			goto end;
		} 
	
		/*����ÿ���ߵ�common neighbor����*/
		for(int i = 1; i <= n; i++)				
		{
			
			for(int j = 1; j <= n; j++)						//�����еı仯 
			{
				First[j] = Clique[i][j].EdgeList;           //  ��ȡ��i��Ԫ�� 
			
			}	
			
			for(int k = 1; k <= n; k++)						//�����еı仯
			{
			
				if(Clique[i][k].EdgeList == 1)				//��֤�������������������� 
				{
				
					for(int j = 1; j <= n; j++)			 
					{
						Second[j] = Clique[k][j].EdgeList;  //  ��ȡ��k��Ԫ�� 
					}
				
					int Diff = 0;								//�ھӼ��� 	
					for(int u = 1; u <= n; u++)
					{
						if(First[u] == Second[u] && First[u] != 0)			//������������(edge) �й�ͬ�ĵ㣨Common_Neignbor��,����0�������� 
						{
								Diff = Diff + 1;
						}	
					}	
					Clique[i][k].Common_Neighbor = Diff;    //ÿ���ߵ�Common_Neighbor���� 
				}
				else
					Clique[i][k].Common_Neighbor = 0;	
			}
		} 

		for(int i = 1; i <= n; i++)				//	ֻ��ȡCommon_Neighbor�����ǲ������ݣ���֤Common_Neighbor[][]ֻ���治ͬ�ߵĸ��� 
		{
			for(int j = 1; j <= n; j++)
			{		
				if(i <= j)
				{
					Clique[i][j].Common_Neighbor_New = Clique[i][j].Common_Neighbor; //��Common_Neighbor����������ȡ���� 
				} 
				if(j == n)
				{	
				} 	
			}	
		} 

		/*******************************************************/
		/***********Common_Neighbor_Sort��������****************/
		/*******************************************************/
		/*��ÿһ���ڵ��Ӧ�����Common_Neighbor�ҳ���*/
		for(int i = 1, j = 1; i <= n; i++)
		{														//*********************֮ǰ����������***************************// 
			int max = Clique[i][i].Common_Neighbor_New;         //�ӶԽ��߿�ʼ  ~~~~~~~~~~~~������ȷ��  ���ֵ����0  ��Ȼ��������ֵ 
			for(j = i; j <= n; j++)
			{
				if(max < Clique[i][j].Common_Neighbor_New)
				{ 
					max = Clique[i][j].Common_Neighbor_New;	//����ǰmaxֵ�� Clique.Common_Neighbor_New[i][j]С�����滻 maxֵ 
				}
			}
			Common_Neighbor_Sort[i] = max;
		}
			
		SelectSort(Common_Neighbor_Sort, n);			//��ͳ�Ƴ�����ÿ��������Common_Neighbor������������ 
	
		/*�����ҳ�Common_Neighbor���ģ���Common_Neighbor���ֵΨһ��������������ֵ��Ψһ���ٱȽ�
		Degree[],���޺ϲ������������ڵ�*/

		int Max_Common_Neighbor_num = 0; 					//ͳ���������Common_Neighbor�ıߵ����� 
		int Max_Common_Neighbor_i_Point[n];					//����ÿ���ߵ�i�ڵ� 
		int Max_Common_Neighbor_j_Point[n];					//����ÿ���ߵ�j�ڵ� 
		int Max_Common_Neighbor_i_Point_Last;				//�����Common_Neighbor�ı߲�ֹһ��ʱ����Ҫ�Ҷ������������ڵ� ��ȡi�ڵ� 
		int Max_Common_Neighbor_j_Point_Last;				//��ȡj�ڵ� 
		int Max_i;											//���ŵ�������~�� ��� 
		int Max_j;	
		Max_Common_Neighbor_num = 0;										//һ������ Max_j > Max_i 
		
		for(int i = 1; i <= n; i++)
		{
			for(int j = i + 1; j <= n; j++)
			{
				// Common_Neighbor��������Ϊ Common_Neighbor_Sort[1]  ÿ�ζ�ֻ�����ĳ����Ƚϼ���  
				if(Common_Neighbor_Sort[1] == Clique[i][j].Common_Neighbor_New && Clique[i][j].EdgeList != 0)	
				//�ҵ����Common_Neighbor�������ڵ� ��ֹ���Common_NeighborΪ0ʱ���ִ��󣬱�֤ѡ�е������ڵ�������� 
				{
					Max_Common_Neighbor_num++;					 						//ͳ�����Common_Neighbor���� 
					Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = i;			//ͳ�����Common_Neighborλ�� ���������ڵ��� 
					Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;			//Max_Common_Neighbor_num������Ϊindex 
				}
			}	
		} 

		Max_Degree = 0;
		if(Max_Common_Neighbor_num != 1)								//�������Common_Neighbor�ı߲�ֹһ�� 
		{
			for(int k = Max_Common_Neighbor_num; k >= 1; k-- )		  //ͨ��Max_Common_Neighbor_num��Ϊindex������һ�����ҳ��� 
			{														  //����ѡ�б��ж������ҳ��� 
				int i = Max_Common_Neighbor_i_Point[k]; 
				int j = Max_Common_Neighbor_j_Point[k];
				if(Max_Degree < (Degree[i] + Degree[j]))				//�ҳ������������� 
				{
					Max_Degree = Degree[i] + Degree[j];
					Max_Common_Neighbor_i_Point_Last = i;				//�Ѿ��ҳ������������� 
					Max_Common_Neighbor_j_Point_Last = j;
				}
			}
			Max_i =  Max_Common_Neighbor_i_Point_Last;					//�Ѿ��ҵ����Common_Neighbor�㼰���ȵ�edge(i,j) 
			Max_j =  Max_Common_Neighbor_j_Point_Last; 
		} 
		else															//���Common_Neighbor�ı�ֻ��һ�� 
		{
			Max_i = Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num]; //�Ѿ��ҵ����Common_Neighbor�㼰���ȵĵ�edge(i,j) 
			Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; //Max_Common_Neighbor_num����index 
		} //end Step1
Step2:
		{  
			//��Max_i ��Max_j ���д��� ���Ƕ�Clique���Ž��д���Node[],NodeList[][]��Ҫ���д���
			//Madx_i ��Max_j ��ʾ���Common_Neighbor �������ȵ��������ӵ� edge(Max_i,Max_j)
			//Node[] == 1ʱ��ʾ�����δ�ܵ�����Node[] == 0ʱ��ʾ�Ѿ�����ĳ��Clique
			//NodeList[i][1] == 1ʱ��ʾ������һ��Clique NodeList[i][1] == 0ʱ��ʾ���ﲻ����Clique  
			
			//Clique[Max_i][Max_j].EdgeList ��Ҫ������ 
			//������Clique[Max_i][]����������Clique[Max_j][]�еĵ��Ϊ0 
			//Clique[Max_j][].EdgeListȫ��Ϊ0
			
			//�Ѿ��õ� Max_i  Max_j Ŀǰ����������������в���  ************************ 
			 
			
			//�Լ���CLique���д���  һ������ Max_i < Max_j  
	
			int temp;
			if(Max_i > Max_j)										//��Max_i�ϴ��򻥻�Max_i��Max_j��ֵ 
			{
				temp  = Max_j;
				Max_j = Max_i;
				Max_i = temp; 
			}
					
			NodeList[Max_j][1] = 0;							     //����һ��CLique
			Node[Max_j]		   = 0;					  	      	//����һ��Node ����ʾ��Max_j��Max_i�ϲ�ΪMax_i 
			
																//����ͬ��һ��������  ����һ������~  
																//���ǵ�ԭ����Max_i > Max_j ʱҪ�� Max_i �� Max_j����
																//NodeList[Max_j][i]��ֵҲ�ñ����Ҫ���浽NodeList[Max_i][i]  
			{
				NodeList[Max_i][Max_j] = Max_j;					//��Ҫ��NodeList[Max_j][1] = Max_j 
				for(int i = 2; i <= n; i++)
				{
					if(NodeList[Max_j][i] != 0)					//������NodeList[Max_j][i]�鲢��NodeList[Max_i][i]�� 
					{
						NodeList[Max_i][i] = i;
					}	
				}
			}		
			for(int i = 1; i <= n; i++)								//NodeList[Max_j][i]�еĵ��Ѿ����鲢��NodeList[Max_i][i]�ˣ���Nodelist[Max_j][i]���� 
			{
				NodeList[Max_j][i] = 0;	
			}											
	
Step3:
			for(int j = 1; j <= n; j++)					     	//�����еı仯 
			{
				First[j] = Clique[Max_i][j].EdgeList;         //  ��ȡ��i��Ԫ�� 
				Second[j] = Clique[Max_j][j].EdgeList;       //  ��ȡ��k��Ԫ�� 
			}	
			for(int u = 1; u <= n; u++)
			{												//�Ѿ���ȷMax_i��Max_j�������ģ��鿴���Ƕ�Ӧ�ڽӾ����Max_i�к�Max_j��
															//��Ӧ���Ƿ���ȣ�����ҵ���1��ʾ��������k��Max_i��Max_j����
															//��Edge(Max_i,k)(���k>Max_iʱ ��ΪEdge(k, Max_i))����Ҫɾ������֮��Ҫɾ�� 
				if(First[u] != Second[u] && First[u] == 1)	//����һ�������������Max_i������ �� Clique[Max_i][u].EdgeList = 1;
				{
						Clique[Max_i][u].EdgeList = 0;		//���ɵ�������ͼ����Ҫ�öԳƵ�����ֵΪ0����Ϊɾ���� 
						Clique[u][Max_i].EdgeList = 0;
				}			
			}
			// �����Ŵ�ĵ㼴Max_j�����ı�ȫ��ɾ��	//����������һЩ�������ȽϺ���⣬����ֱ��˵��ͨ 
			for(int i = 1; i <= n; i++)
			{
				Clique[Max_j][i].EdgeList = 0;			//���ɵ�������ͼ����Ҫ�öԳƵ�����ֵΪ0����Ϊɾ���� 
				Clique[i][Max_j].EdgeList = 0;
			}
			
Step4:
			Max_i;									 	//Max_i �� Max_j �ϲ�Ϊ Max_i 
		} 
	
Step5:
		/*���ڽڵ㱻�޸ģ�ÿ���ڵ�Ķ�Ҳ���޸��� ���¼���ȵ�ֵ*/ 
			for(int i = 1; i <= n; i++)
			{
				Degree[i] = 0;										//ÿ���ڵ�Ķȳ�ʼ��Ϊ0 
				for(int j = 1; j <= n; j++)
				{
					Degree[i] = Degree[i] + Clique[i][j].EdgeList; // ÿһ�е�ֵ���ܺͼ�Ϊĳ���ڵ�Ķ� 
				}	
				Degree_Inceasing_Order[i] = Degree[i];				//��Degree[]��ֵ���δ���Degree_Increasing_Order[]��������
																	//����Ҫ����Degree���䣬���� ����ѯʱ�ڵ㡱 ʱʹ�� 
			} 
			/*��ӡÿ���ڵ�Ķ�*/
										//���ڵ�ĶȽ�������~ 
			SelectSort(Degree_Inceasing_Order, n);
			
			max_Degree_Inceasing_Order = Degree_Inceasing_Order[1];

			if(max_Degree_Inceasing_Order == 0)						//�Ի�ȡ���ڵ��Ϊ0��������ֹ���� 
			{
				goto end;
			}  	

			if(Degree[Max_i] == 0)					//���Max_i�ڵ�����ˣ���ѡһ���߼����������� 
			{
				goto Step1;
			}
			else								    // ������Max_i�����ı� 
			{
				/*�������Max_i�ڵ�ıߵ�Common_Neighbor�����ֵ���*/	
				for(int j = 1; j <= n; j++)						//�����еı仯 
				{
					First[j] = Clique[Max_i][j].EdgeList;           //  ��ȡ��i��Ԫ�� 
				}	
				for(int k = 1; k <= n; k++)						//�����еı仯
				{
					if(Clique[Max_i][k].EdgeList == 1)				//��֤�������������������� 
					{
						for(int j = 1; j <= n; j++)			 
						{
							Second[j] = Clique[k][j].EdgeList;  //  ��ȡ��k��Ԫ�� 
						}
						int Diff = 0;								//�ھӼ��� 	
						for(int u = 1; u <= n; u++)
						{
							if(First[u] == Second[u] && First[u] != 0)			//������������ �й�ͬ�ĵ㣨Common_Neignbor��,����0�������� 
							{
									Diff = Diff + 1;
							}	
						}	
						Clique[Max_i][k].Common_Neighbor = Diff;    //ÿ���ߵ�Common_Neighbor���� 
					}
					else
						Clique[Max_i][k].Common_Neighbor = 0;	
				}
				//	ֻ��ȡCommon_Neighbor�����ǲ������ݣ���֤Common_Neighbor[][]ֻ���治ͬ�ߵĸ��� 
				for(int j = 1; j <= n; j++)						//��֪��Max_i �� j˭��˭С 
				{
					Clique[Max_i][j].Common_Neighbor_New = Clique[Max_i][j].Common_Neighbor; //��Common_Neighbor����������ȡ���� 
				}	
	
				//�Ѿ���� Clique[Max_i][k].Common_Neighbor_New	
				int max = Clique[Max_i][1].Common_Neighbor_New; 
				for(int j = 2; j <= n; j++)
				{
					if(max < Clique[Max_i][j].Common_Neighbor_New)
					{ 
						max = Clique[Max_i][j].Common_Neighbor_New;	//����ǰmaxֵ�� Clique.Common_Neighbor_New[i][j]С�����滻 maxֵ 
					}
				}
				Common_Neighbor_Sort[1] = max;
				// Max_i����Common_Neighbor Ϊ max 
				//����Common_Neighbor = max
				//��������ֱ�Ϊ Max_i Max_j �ж�max �Ƿ�Ψһ
				Max_Common_Neighbor_num = 0;
				for(int j = 1; j <= n; j++)
				{
					// Common_Neighbor��������Ϊ Common_Neighbor_Sort[1]  ÿ�ζ�ֻ�����ĳ����Ƚϼ���  
					if(Common_Neighbor_Sort[1] == Clique[Max_i][j].Common_Neighbor_New && Clique[Max_i][j].EdgeList != 0)	
					//�ҵ����Common_Neighbor�������ڵ� ��ֹ���Common_NeighborΪ0ʱ���ִ��󣬱�֤ѡ�е������ڵ�������� 
					{
						Max_Common_Neighbor_num++;					 						//ͳ�����Common_Neighbor���� 
						//Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = Max_i;			//ͳ�����Common_Neighborλ�� ���������ڵ��� 
						Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;			//Max_Common_Neighbor_num������Ϊindex 
					}
				}	
				Max_Degree = 0;
				if(Max_Common_Neighbor_num != 1)								//�������Common_Neighbor�ı߲�ֹһ�� 
				{
					for(int k = Max_Common_Neighbor_num; k >= 1; k-- )		  //ͨ��Max_Common_Neighbor_num��Ϊindex������һ�����ҳ��� 
					{														  //����ѡ�б��ж������ҳ��� 
						 
						int j = Max_Common_Neighbor_j_Point[k];
						if(Max_Degree < (Degree[Max_i] + Degree[j]))				//�ҳ������������� 
						{
							Max_Degree = Degree[Max_i] + Degree[j];
																					//�Ѿ��ҳ������������� 
							Max_Common_Neighbor_j_Point_Last = j;
						}
					}
																					//�Ѿ��ҵ����Common_Neighbor�㼰���ȵ�edge(i,j) 
					Max_j =  Max_Common_Neighbor_j_Point_Last; 
				} 
				else															//���Common_Neighbor�ı�ֻ��һ�� 
				{
																				 //�Ѿ��ҵ����Common_Neighbor�㼰���ȵĵ�edge(i,j) 
					Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; //Max_Common_Neighbor_num����index 
				} 
				{
					//���»�� Max_i �� Max_j �����ڵ�Ϊ����ѡ��� 
					//Max_i;
					//Max_j;
					goto Step2; 
				} 
			}			
	}//end while(Max_Degree_Increasing_Order)
	
	//#ifdef Debug 
		/*����Clique�ĸ���*/
		cout << "******************" << endl;
		for(int i = 1; i <= n; i++)
		{
			if(NodeList[i][1] != 0)					//Clique������NodeList[][]�� NodeList[i][1] = 1��ʾ������һ����
													//NodeList[i][1] = 0��ʾ����û���� 
			{
				Clique_Count++;	
			} 
		} 
		cout << "print Clique_Count = " << Clique_Count << endl; 
		for(int i = 1; i <= n; i++)
		{
			if(NodeList[i][1] != 0)									//��֤���ĸ��ڵ�һ������~ 
			{
				cout << "Clique[" << i << "]:( " << i << ",";		//�ڼ�������л�������Щ�ڵ� 
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
	//#endif
		cout << "Do you want to continue? ��1�� Yes ��2�� No" << endl;
		int flag_go_on;
		cin >> flag_go_on;
		if(flag_go_on == 1)
		{
			goto go_on;
		}
		return 0;
	}

int SelectSort(int *A, int n)				//����һ��������
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
