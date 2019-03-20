/*
程序功能：实现摄氏和华氏温度的转换
该问题主要是找到两种温度转换的公式
注意为浮点输入，浮点输出且保留一位小数 
*/ 
#include <stdio.h> 
int main()
{
	// 定义摄氏度和华氏度 
	float C, F;
	// 程序是否继续的标志
	// ContinueFlag == 1表示继续
	// ContinueFlag == 其它值表示程序结束 
	int ContinueFlag = 1;
	// 为了重复重复执行程序 
	while(ContinueFlag == 1)
	{
		printf("Please Input Celsius C = ");
		// 输入摄氏度 
		scanf("%f", &C);
		// 摄氏度和华氏度的转换公式 
		F = (9.0/5.0)*C + 32.0; 
		// 输出2华氏度 
		printf("Output Fahrenheit F = %0.1f \n", F);
		printf("\nDo you want to continue? 【0】No 【1】Yes \n");
		printf("Your Answer is ="); 
		// 修改程序是否执行的标志 
		scanf("%d", &ContinueFlag); 
	}
	printf("Thanks for your use. See you~"); 
	return 0; 
} 
