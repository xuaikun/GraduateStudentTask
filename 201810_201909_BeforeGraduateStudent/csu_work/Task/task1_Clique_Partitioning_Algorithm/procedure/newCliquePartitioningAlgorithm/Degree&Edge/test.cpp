#include<iostream>
#include<fstream>
#include<typeinfo>
using namespace std;
int main()
{
	float data[6][6] = { 0 };//����һ��1500*2�ľ������ڴ������
	ifstream infile;//�����ȡ�ļ���������ڳ�����˵��in
	infile.open("what1.txt");//���ļ�
	for (int i = 1; i <= 5; i++)//������ѭ��
	{
		for (int j = 1; j <= 5; j++)//������ѭ��
		{
			infile >> data[i][j];//��ȡһ��ֵ���ո��Ʊ�������и�������д�뵽�����У����в���ѭ������
		}
	}
	infile.close();//��ȡ���֮��ر��ļ�
	for(int i = 1; i <= 5; i++)
	{
		for(int j = 1; j <= 5; j++)
		{
			cout << data[i][j] << " ";
			if(j == 5)
			{
				cout << endl;
			}
		}
	}
	return 0;
}

