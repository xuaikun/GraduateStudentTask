/*
�������ܣ������Сд��ϵ��ַ�����ɾ����д��ĸ������Сд��ĸ 
����:char[20]������
���:char[20]������ 
�������룺PalmaNonSinePulvere
���������almaonineulvere
*/
#include <stdio.h>
#include <conio.h>
int main()
{
	// ��������Ĵ�С 
	int charNum = 20;
	// �����Ƿ�����ı�־
	// ContinueFlag == 1��ʾ����
	// ContinueFlag == ����ֵ��ʾ������� 
	int ContinueFlag = 1;
	// Ϊ���ظ��ظ�ִ�г��� 
	while(ContinueFlag == 1)
	{
		//��ʼ���������� 
		char x[charNum] ={};
		int i = 0;
		printf("Please Input the string\n");
		// �����ַ�������Ϊ���� 
		while(1)
		{
			// ������ʱ���������ڴ�ŵ�ǰ������ַ� 
			char temp;
			temp = getche();
			// ���뻻����������� 
			if (temp - '\n' == 3)
			{
				// �˳���ǰ������� 
				break;
			}
			// �����Ϳ��Ա����˽����з�����ӵ����������� 
			x[i] = temp;
			i++;
			// ����������charNum���ַ�
			if(i == charNum)
			{
				// �˳���ǰ������� 
				break;
			}
		} 
		// ����ˢ�������� 
		printf("\n");
		// iΪ�����index 
		i = 0;
		// jΪ�������index 
		int j = 0;
		while(1)
		{ 
			// ֻ��ȡСд��ĸ 
			// 'a'->97 'z'->122 '0'->48 
			if(x[i] - '0' >= 49 && x[i] - '0' <= 74)
			{
				j++;
			}
			// index��1 
			i++;
			if(i == charNum)
			{
				//printf("�ַ���ȡ���\n");
				break;	
			} 
		}
		//��ʼ��������� ,jΪͳ��С��ĸ�ĸ��� 
		char x_new[j];
		// iΪ���������index 
		i = 0;
		// kΪ��������index 
		int k = 0;
		while(1)
		{ 
			// ֻ��ȡСд��ĸ 
			// 'a'->97 'z'->122 '0'->48 
			if(x[i] - '0' >= 49 && x[i] - '0' <= 74)
			{
				x_new[k] = x[i];
				k++;
			}
			// index��1 
			i++; 
			if(i == charNum)
			{
				//printf("�ַ���ȡ���\n");
				break;	
			} 
		}
		// ������ֻ��ӡСд��ĸ 
		printf("%s\n\n\n", x_new);
		printf("Do you want to continue? ��0��No ��1��Yes \n");
		printf("Your Answer is ="); 
		// �޸ĳ����Ƿ�ִ�еı�־ 
		scanf("%d", &ContinueFlag); 
	}
	printf("Thanks for your use. See you~");
	return 0;
}
