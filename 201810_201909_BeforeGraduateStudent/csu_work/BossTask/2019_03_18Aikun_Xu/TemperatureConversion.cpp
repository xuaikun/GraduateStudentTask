/*
�����ܣ�ʵ�����Ϻͻ����¶ȵ�ת��
��������Ҫ���ҵ������¶�ת���Ĺ�ʽ
ע��Ϊ�������룬��������ұ���һλС�� 
*/ 
#include <stdio.h> 
int main()
{
	// �������϶Ⱥͻ��϶� 
	float C, F;
	// �����Ƿ�����ı�־
	// ContinueFlag == 1��ʾ����
	// ContinueFlag == ����ֵ��ʾ������� 
	int ContinueFlag = 1;
	// Ϊ���ظ��ظ�ִ�г��� 
	while(ContinueFlag == 1)
	{
		printf("Please Input Celsius C = ");
		// �������϶� 
		scanf("%f", &C);
		// ���϶Ⱥͻ��϶ȵ�ת����ʽ 
		F = (9.0/5.0)*C + 32.0; 
		// ���2���϶� 
		printf("Output Fahrenheit F = %0.1f \n", F);
		printf("\nDo you want to continue? ��0��No ��1��Yes \n");
		printf("Your Answer is ="); 
		// �޸ĳ����Ƿ�ִ�еı�־ 
		scanf("%d", &ContinueFlag); 
	}
	printf("Thanks for your use. See you~"); 
	return 0; 
} 
