/*
����ʵ�ֹ��ܣ�ͨ���������ռ�����˶�����
���������Ϊ����
ע��ĵ㣺�����꣬��Ҫ֪�����о�������Щ���꣬�����2����29��
��ƽ���һ��
������366��
ƽ����365��
*/
#include <stdio.h>
#include <math.h>
// ������������������� 
int TodayYear = 2019;
int TodayMonth = 3;
int TodayDay = 15;
int RunYear[13] = {0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int PingYear[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int main()
{
	// ���������� 
	int Year, Month, Day;
	// �����Ƿ�����ı�־
	// ContinueFlag == 1��ʾ����
	// ContinueFlag == ����ֵ��ʾ������� 
	int ContinueFlag = 1;
	// Ϊ���ظ��ظ�ִ�г��� 
	while(ContinueFlag == 1)
	{
		int DayAgeSum = 0; 
		printf("Input Birthday: \nYear Month Day\n");
		// ������������� 
		scanf("%d%d%d", &Year, &Month, &Day);
		int tempYear; 
		tempYear = Year + 1; 
		// ���ӳ����ĵڶ��꿪ʼ���㣬ֱ��ȥ��Ϊֹ�����㾭���˶������� 
		while((tempYear != TodayYear)&&(tempYear <= TodayYear))
		{
			// ����ȫ��366�� 
			if ((tempYear%4 == 0&& tempYear%100 != 0)||(tempYear%400 == 0))
			{
				DayAgeSum = DayAgeSum + 366; 
			} 
			// ƽ��ȫ��365�� 
			else
			{
				DayAgeSum = DayAgeSum + 365;
			}
			// ������һ�� 
			tempYear++;
		}
		
		// ��λ��ȥ�� 
		Year = tempYear - 1;  // �� 
		Month = Month;		//	�� 
		Day = Day; 			// ��
		// �����·ݱȵ�ǰ�·�С����һ�꣬������366���Ǽ�365 
		if(Month < TodayMonth)
		{
			// printf("�����·�С�ڵ�ǰ�·�\n");
			// �ж��Ƿ�Ϊ2��֮����·ݳ��� 
			if (Month <= 2) 
			{
				// �ж�ȥ���Ƿ�Ϊ���� 
				if ((Year%4 == 0&& Year%100 != 0)||(Year%400 == 0))
				{
					// printf("%d ������ \n" , Year);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					// �����ĵ�ǰ��--������������ 
					for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
					{
						// ��ͬһ����� 
						if (TodayYear == Year)
						{
							DayAgeSum = DayAgeSum + RunYear[tempMonth];
						}
						// �Բ�ͬ����� 
						else{
							DayAgeSum = DayAgeSum + PingYear[tempMonth];
						}
						
					} 
				} 
				// ȥ�겻Ϊ���� 
				else 
				{
					// �жϽ����Ƿ�Ϊ���� 
					if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
					{
						// printf("%d ������ \n" , TodayYear);
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						// �����ĵ�ǰ��--������������ 
						for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
						{
							// ����ȥ��Day�ǳ����������
							DayAgeSum = DayAgeSum + RunYear[tempMonth];
						} 
					}
					else
					{
						//��һ�ּ��㷽�� 
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						// �����ĵ�ǰ��--������������ 
						for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
						{
							// ����ȥ��Day�ǳ����������
							DayAgeSum = DayAgeSum + PingYear[tempMonth];
						}
					}
					
				} 
			}
			// month > 2,��֮ǰ���� 
			else
			{
				// ������·�һ������3 
				// �жϽ����Ƿ�Ϊ���� 
				if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
				{
					// printf("%d ������ \n" , TodayYear);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					// �����ĵ�ǰ��--������������ 
					for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
					{
						// ����ȥ��Day�ǳ����������, ���������� 
						DayAgeSum = DayAgeSum + PingYear[tempMonth];
					} 
				} 
				else
				{
					//��һ�ּ��㷽�� 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
					// �����ĵ�ǰ��--������������ 
					for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
					{
						// ����ȥ��Day�ǳ���������գ�������ƽ�� 
						DayAgeSum = DayAgeSum + PingYear[tempMonth];
					}
				} 	
			}
			// ���Ͻ������, �ѳ���������� 
			DayAgeSum = abs(DayAgeSum + TodayDay - Day);
		}
		// �����·ݵ��ڵ�ǰ�·� 
		else if(Month == TodayMonth)
		{
			// printf("�����·��뵱ǰ�·����");
			// Month<= 2ʱ��ֻ��Ҫ����ȥ���ǲ������� 
			if(Month <= 2)
			{
				// �ж�ȥ���Ƿ�Ϊ���� 
				if ((Year%4 == 0&& Year%100 != 0)||(Year%400 == 0))
				{
					// printf("%d ������ \n" , Year);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
				} 
				else
				{
					//��һ�ּ��㷽�� 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
				} 
			 } 
			 // Month > 2ʱֻ��Ҫ���ǽ����ǲ������� 
			 else
			 {
			 	// �ж�ȥ���Ƿ�Ϊ���� 
				if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
				{
					// printf("%d ������ \n" , TodayYear);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
				} 
				else
				{
					//��һ�ּ��㷽�� 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
				} 
			 } 
			// ����������������ռ��Ͻ������ 
			DayAgeSum = abs(DayAgeSum + TodayDay - Day);
		} 
		// �����·ݴ��ڵ�ǰ�·� 
		else if(Month > TodayMonth)
		{
			// printf("�����·ݴ��ڵ�ǰ�·�\n");
			// �жϽ����Ƿ񳬹�2�£�����2�µÿ����Ƿ������� 
			if(TodayMonth > 2)
			{
				// �жϽ����Ƿ�Ϊ���� 
				if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
				{
					// printf("%d ������ \n" , TodayYear);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					
					int tempMonth;
					// Ӧ�ô���һ���¿�ʼ ��  
					for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
					{
						// ���������꣬����������Ӧ��������� 
						DayAgeSum = abs(DayAgeSum - RunYear[tempMonth]);
					}
					// �Ҽ���ķ�ʽ�ǽ������ӳ����磺������2018��10��16��������2019��3��15
					//  ���������2019��10��10�գ��Ǽ���ͼ��ˣ�һ��ʱ��-��10��16�յ�3��15�������� 
					// ���������3��15�� 
					// ������10��16�� 
					// ���������꣬����������Ӧ��������� 
					DayAgeSum = abs(DayAgeSum - (RunYear[TodayMonth] - TodayDay) - Day);
				} 
				else
				{
					//��һ�ּ��㷽�� 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
					
					int tempMonth;
					// Ӧ�ô���һ���¿�ʼ ��  
					for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
					{
						// ������ƽ�꣬����������Ӧ����ƽ��� 
						DayAgeSum = abs(DayAgeSum - PingYear[tempMonth]);
					}
					// �Ҽ���ķ�ʽ�ǽ������ӳ����磺������2018��10��16��������2019��3��15
					//  ���������2019��10��10�գ��Ǽ���ͼ��ˣ�һ��ʱ��-��10��16�յ�3��15�������� 
					// ���������3��15�� 
					// ������10��16�� 
					// ������ƽ�꣬����������Ӧ����ƽ��� 
					DayAgeSum = abs(DayAgeSum - (PingYear[TodayMonth] - TodayDay) - Day);
				} 
				
			} 
			// ˵����2��ǰ�������ÿ���ȥ����Ƿ������� 
			if(Month <= 2)
			{
				// �ж�ȥ���Ƿ�Ϊ���� 
				if ((Year%4 == 0&& Year%100 != 0)||(Year%400 == 0))
				{
					// printf("%d ������ \n" , Year);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					int tempMonth;
					// Ӧ�ô���һ���¿�ʼ ��  
					for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
					{
						// ȥ�������꣬����������Ӧ����ƽ��� 
						DayAgeSum = abs(DayAgeSum - PingYear[tempMonth]);
					}
					// �Ҽ���ķ�ʽ�ǽ������ӳ����磺������2018��10��16��������2019��3��15
					//  ���������2019��10��10�գ��Ǽ���ͼ��ˣ�һ��ʱ��-��10��16�յ�3��15�������� 
					// ���������3��15�� 
					// ������10��16�� 
					
					DayAgeSum = abs(DayAgeSum - (PingYear[TodayMonth] - TodayDay) - Day);
				} 
				else
				{
					// �жϽ����Ƿ�Ϊ���� 
					if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
					{
						// printf("%d ������ \n" , TodayYear);
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						int tempMonth;
						// Ӧ�ô���һ���¿�ʼ ��  
						for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
						{
							// ����Ϊ���꣬��ȥ������������� 
							DayAgeSum = abs(DayAgeSum - RunYear[tempMonth]);
						}
						// �Ҽ���ķ�ʽ�ǽ������ӳ����磺������2018��10��16��������2019��3��15
						//  ���������2019��10��10�գ��Ǽ���ͼ��ˣ�һ��ʱ��-��10��16�յ�3��15�������� 
						// ���������3��15�� 
						// ������10��16�� 
						// ����Ϊ���꣬��ȥ�������������
						DayAgeSum = abs(DayAgeSum - (RunYear[TodayMonth] - TodayDay) - Day);
					} 
					else
					{
						//��һ�ּ��㷽�� 
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						int tempMonth;
						// Ӧ�ô���һ���¿�ʼ ��  
						for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
						{
							// ����Ϊƽ�꣬��ȥ����ƽ�������
							DayAgeSum = abs(DayAgeSum - PingYear[tempMonth]);
						}
						// �Ҽ���ķ�ʽ�ǽ������ӳ����磺������2018��10��16��������2019��3��15
						//  ���������2019��10��10�գ��Ǽ���ͼ��ˣ�һ��ʱ��-��10��16�յ�3��15�������� 
						// ���������3��15�� 
						// ������10��16�� 
						// ����Ϊƽ�꣬��ȥ����ƽ�������
						DayAgeSum = abs(DayAgeSum - (PingYear[TodayMonth] - TodayDay) - Day);
					}
				} 
			}	
		} 
		printf("%d\n", DayAgeSum);
		printf("\nDo you want to continue? ��0��No ��1��Yes \n");
		printf("Your Answer is ="); 
		// �޸ĳ����Ƿ�ִ�еı�־ 
		scanf("%d", &ContinueFlag); 
	}
	printf("Thanks for your use. See you~");
	return 0;
}
