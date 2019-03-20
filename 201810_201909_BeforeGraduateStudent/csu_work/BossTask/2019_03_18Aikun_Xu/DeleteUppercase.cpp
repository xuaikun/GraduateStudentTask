/*
函数功能：输入大小写混合的字符串，删除大写字母，留下小写字母 
输入:char[20]型数组
输出:char[20]型数组 
样例输入：PalmaNonSinePulvere
样例输出：almaonineulvere
*/
#include <stdio.h>
#include <conio.h>
int main()
{
	// 输入数组的大小 
	int charNum = 20;
	// 程序是否继续的标志
	// ContinueFlag == 1表示继续
	// ContinueFlag == 其它值表示程序结束 
	int ContinueFlag = 1;
	// 为了重复重复执行程序 
	while(ContinueFlag == 1)
	{
		//初始化输入数组 
		char x[charNum] ={};
		int i = 0;
		printf("Please Input the string\n");
		// 输入字符，换行为结束 
		while(1)
		{
			// 定义临时变量，用于存放当前输入的字符 
			char temp;
			temp = getche();
			// 输入换行则结束输入 
			if (temp - '\n' == 3)
			{
				// 退出当前输入操作 
				break;
			}
			// 这样就可以避免了将换行符号添加到数组里面了 
			x[i] = temp;
			i++;
			// 最多可以输入charNum个字符
			if(i == charNum)
			{
				// 退出当前输入操作 
				break;
			}
		} 
		// 类似刷新命令行 
		printf("\n");
		// i为数组的index 
		i = 0;
		// j为新数组的index 
		int j = 0;
		while(1)
		{ 
			// 只提取小写字母 
			// 'a'->97 'z'->122 '0'->48 
			if(x[i] - '0' >= 49 && x[i] - '0' <= 74)
			{
				j++;
			}
			// index加1 
			i++;
			if(i == charNum)
			{
				//printf("字符提取完毕\n");
				break;	
			} 
		}
		//初始化输出数组 ,j为统计小字母的个数 
		char x_new[j];
		// i为输入数组的index 
		i = 0;
		// k为输出数组的index 
		int k = 0;
		while(1)
		{ 
			// 只提取小写字母 
			// 'a'->97 'z'->122 '0'->48 
			if(x[i] - '0' >= 49 && x[i] - '0' <= 74)
			{
				x_new[k] = x[i];
				k++;
			}
			// index加1 
			i++; 
			if(i == charNum)
			{
				//printf("字符提取完毕\n");
				break;	
			} 
		}
		// 按道理只打印小写字母 
		printf("%s\n\n\n", x_new);
		printf("Do you want to continue? 【0】No 【1】Yes \n");
		printf("Your Answer is ="); 
		// 修改程序是否执行的标志 
		scanf("%d", &ContinueFlag); 
	}
	printf("Thanks for your use. See you~");
	return 0;
}
