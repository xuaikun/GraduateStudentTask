#include<iostream>
#include<fstream>
#include<typeinfo>
using namespace std;
int main()
{
	float data[6][6] = { 0 };//定义一个1500*2的矩阵，用于存放数据
	ifstream infile;//定义读取文件流，相对于程序来说是in
	infile.open("what1.txt");//打开文件
	for (int i = 1; i <= 5; i++)//定义行循环
	{
		for (int j = 1; j <= 5; j++)//定义列循环
		{
			infile >> data[i][j];//读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
		}
	}
	infile.close();//读取完成之后关闭文件
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

