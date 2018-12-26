#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
//#define Debug                              //�����õ� 
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
	int n;                                //����ڵ���� 
	int Clique_Count = 0;				  // ���Ÿ��� 
	cout << "����ڵ������";
	cin >> n;                            //����ڵ���� 
	int Node[n];                         //���ڼ���ĳ���ڵ��Ƿ�û�м���ĳ���� 
	int NodeList[n][n];                 //���ڵ㶼�������ĸ�NodeList����¼�����˼���NodeList 
	int First[n];						//���������м������� 
	int Second[n];
	int Common_Neighbor_Sort[n];		//Common_Neighbor����
	int Degree[n];						//ÿ���ڵ�Ķ� 
	int Degree_Inceasing_Order[n];		//�Ѷȸ����� 
	int Max_Degree = 0;												//���ȳ�ʼ��Ϊ1,Ҳ����ֹ������������Ϊ0ʱ����ֹ 
	for(int i = 1; i <= n; i++)          //��ʼ���ڵ� 
	{
		Node[i]        = 1;               //ȷ��ÿ���ڵ㶼���� 
		NodeList[i][1] = 1;               //ȷ����ʼ��ʱÿ���ڵ������һ��Clique��
		for(int j = 2; j <= n; j++)      //ÿ��NodeList��ʼ��ʱֻ��NodeList[i][0] = 1��i != 0ʱ NodeList[i][j] = 0; 
		{
			NodeList[i][j] = 0;
		} 
 	}
 	/*��ӡ�ڵ����ŵ����*/
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
	cout << "��ʼ��max_Degree_Inceasing_Order = " <<max_Degree_Inceasing_Order << endl;
	cout << "��ʼ����ӡClique[][].EdgeList����" << endl;
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
	/*���ÿ���ڵ�Ķ�~*/
	for(int i = 1; i <= n; i++)
	{
		Degree[i] = 0;										//ÿ���ڵ�Ķȳ�ʼ��Ϊ0 
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
					if(First[u] == Second[u] && First[u] != 0)			//������������ �й�ͬ�ĵ㣨Common_Neignbor��,����0�������� 
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
#ifdef Debug
	cout << "****************" << endl;
	cout << "Common_Neighbor" << endl;
	/*cout << "ȫ��" << endl;
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
	cout << "Clique[i][j].Common_Neighbor_New:������" << endl;
#endif
	for(int i = 1; i <= n; i++)				//	ֻ��ȡCommon_Neighbor�����ǲ������ݣ���֤Common_Neighbor[][]ֻ���治ͬ�ߵĸ��� 
	{
		for(int j = i; j <= n; j++)
		{
#ifdef Debug
			cout <<Clique[i][j].Common_Neighbor << " ";
#endif
			Clique[i][j].Common_Neighbor_New = Clique[i][j].Common_Neighbor; //��Common_Neighbor����������ȡ���� 
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
				max = Clique[i][j].Common_Neighbor_New;	//����ǰmaxֵ�� Clique.Common_Neighbor_New[i][j]С�����滻 maxֵ 
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
	cout << "Common_Neighbor_Sort:" ;							//�Ѿ�������Common_Neighbor�ҳ����� ��increasing order�� 
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

	/*�����ҳ�Common_Neighbor���ģ���Common_Neighbor���ֵΨһ��������������ֵ��Ψһ���ٱȽ�
	Degree[],���޺ϲ������������ڵ�*/
#ifdef Debug 
	cout << "****************" << endl;
#endif
	int Max_Common_Neighbor_num = 0; 
	int Max_Common_Neighbor_i_Point[n];
	int Max_Common_Neighbor_j_Point[n];
	int Max_Common_Neighbor_i_Point_Last;
	int Max_Common_Neighbor_j_Point_Last;
	int Max_i;
	int Max_j;											//һ����� Max_j > Max_i 
	for(int i = 1; i <= n; i++)
	{
		for(int j = i + 1; j <= n; j++)
		{
			// Common_Neighbor��������Ϊ Common_Neighbor_Sort[1]  
			if(Common_Neighbor_Sort[1] == Clique[i][j].Common_Neighbor_New && Clique[i][j].EdgeList != 0)	//�ҵ����Common_Neighbor�������ڵ� 
			{
				Max_Common_Neighbor_num++;					 						//ͳ�����Common_Neighbor���� 
				Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] = i;				//ͳ�����Common_Neighborλ�� 
				Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] = j;
			}
		}	
	} 
	if(Max_Common_Neighbor_num != 1)								//��ֹһ�����Common_Neighbor��
	{
#ifdef Debug
		cout << "Common_Neighbor�����㲻ֹһ��" << endl; 
#endif
	 
		for(int k = Max_Common_Neighbor_num; k >= 1; k-- )
		{
			int i = Max_Common_Neighbor_i_Point[k]; 
			int j = Max_Common_Neighbor_j_Point[k];
#ifdef Debug
			cout << "i=" << i << endl; 
			cout << "j=" << j << endl; 
#endif
			if(Max_Degree < (Degree[i] + Degree[j]))				//�ҳ������������� 
			{
				Max_Degree = Degree[i] + Degree[j];
				Max_Common_Neighbor_i_Point_Last = i;				//�Ѿ��ҳ������������� 
				Max_Common_Neighbor_j_Point_Last = j;
			}
		}

	
		//Max_i = Max_Common_Neighbor_i_Point_Last						//�Ѿ��ҵ����Common_Neighbor�㼰���ȵ�edge(i,j) 
		//Max_j = Max_Common_Neighbor_j_Point_Last
#ifdef Debug
		cout << "���ȵ�Common_Neighbor�ĵ�"<<endl;
		cout << "Max_i=" << Max_Common_Neighbor_i_Point_Last << endl;
		cout << "Max_j=" << Max_Common_Neighbor_j_Point_Last << endl;
#endif
		Max_i =  Max_Common_Neighbor_i_Point_Last;
		Max_j =  Max_Common_Neighbor_j_Point_Last; 
	} 
	else															//����һ��Common_Neighbor��
	{
#ifdef Debug
		cout << "Common_Neighbor�������Ϊһ��" << endl; 
		//Max_i = Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] //�Ѿ��ҵ����Common_Neighbor�㼰���ȵĵ�edge(i,j) 
		//Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]
		cout << "Max_i=" << Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num] << endl; 
		cout << "Max_j=" << Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num] << endl; 
#endif
		Max_i = Max_Common_Neighbor_i_Point[Max_Common_Neighbor_num];
		Max_j = Max_Common_Neighbor_j_Point[Max_Common_Neighbor_num]; 
	} 
	{ 
	//��Max_i ��Max_j ���д��� ���Ƕ�Clique���Ž��д���Node[],NodeList[][]��Ҫ���д���
	//Madx_i ��Max_j ��ʾ���Common_Neighbor �������ȵ��������ӵ� edge(Max_i,Max_j)
	//Node[] == 1ʱ��ʾ�����δ�ܵ�����Node[] == 0ʱ��ʾ�Ѿ�����ĳ��Clique
	//NodeList[i][1] == 1ʱ��ʾ������һ��Clique NodeList[i][1] == 0ʱ��ʾ���ﲻ����Clique  
	
	//Clique[Max_i][Max_j].EdgeList ��Ҫ������ 
	//������Clique[Max_i][]����������Clique[Max_j][]�еĵ��Ϊ0 
	//Clique[Max_j][].EdgeListȫ��Ϊ0
	
	//Max_i  Max_j Ŀǰ����������������в��� 
	
	//�Լ���CLique���д��� 
	NodeList[Max_j][1] = 0;											//����һ��CLique
	Node[Max_j]		   = 0;											//����һ��Node
	
	//�Ա߽��в��� ɾ������Max_i�������ı� 

	for(int j = 1; j <= n; j++)						//�����еı仯 
	{
		First[j] = Clique[Max_i][j].EdgeList;           //  ��ȡ��i��Ԫ�� 
		Second[j] = Clique[Max_j][j].EdgeList;  //  ��ȡ��k��Ԫ�� 
	}	
	for(int u = 1; u <= n; u++)
	{
		if(First[u] != Second[u] && First[u] == 1)			//����һ�������������Max_i������ 
		{
				Clique[Max_i][u].EdgeList = 0;
				Clique[u][Max_i].EdgeList = 0;
		}			
	}
	//����Max_j�����ı߽���ɾ�� 
	for(int i = 1; i <= n; i++)
	{
		Clique[Max_j][i].EdgeList = 0;	
		Clique[i][Max_j].EdgeList = 0;
	}
	
	
#ifdef Debug 	
	cout << "���´�ӡClique[][].EdgeList����" << endl;
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
	cout << "����max_Degree_Inceasing_Order = " <<max_Degree_Inceasing_Order << endl;
	//_sleep(5000);
#endif

}
//#ifdef Debug 
	/*����Clique�ĸ���*/
	cout << "******************" << endl;
	for(int i = 1; i <= n; i++)
	{
		if(NodeList[i][1] != 0)					//Clique������NodeList[][]�� 
		{
			Clique_Count++;	
		} 
	} 
	cout << "print Clique_Count = " << Clique_Count << endl; 
//#endif
/*
�����Ѿ������������ͼ G,����������ͼG��Ӧ����С�Ż����㷨
�����ҵ��ڵ��������������
�����бߴ��ڵ�ʱ�򣬾�Ҫ�������� 
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
//ͳ�ƶ���
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
	cout << "Do you want to continue? ��1�� Yes ��2�� No" << endl;
	int flag_go_on;
	cin >> flag_go_on;
	if(flag_go_on == 1)
	{
		goto go_on;
	}
	return 0;
}
