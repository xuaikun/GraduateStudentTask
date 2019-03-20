/*
程序实现功能：通过输入生日计算活了多少日
输入输出都为整形
注意的点：有闰年，需要知道其中经历了哪些闰年，闰年的2月有29日
比平年多一天
闰年有366天
平年有365天
*/
#include <stdio.h>
#include <math.h>
// 今天的日历：测试日期 
int TodayYear = 2019;
int TodayMonth = 3;
int TodayDay = 15;
int RunYear[13] = {0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int PingYear[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int main()
{
	// 定义年月日 
	int Year, Month, Day;
	// 程序是否继续的标志
	// ContinueFlag == 1表示继续
	// ContinueFlag == 其它值表示程序结束 
	int ContinueFlag = 1;
	// 为了重复重复执行程序 
	while(ContinueFlag == 1)
	{
		int DayAgeSum = 0; 
		printf("Input Birthday: \nYear Month Day\n");
		// 输入出生年月日 
		scanf("%d%d%d", &Year, &Month, &Day);
		int tempYear; 
		tempYear = Year + 1; 
		// 将从出生的第二年开始计算，直到去年为止，计算经过了多少闰年 
		while((tempYear != TodayYear)&&(tempYear <= TodayYear))
		{
			// 闰年全年366天 
			if ((tempYear%4 == 0&& tempYear%100 != 0)||(tempYear%400 == 0))
			{
				DayAgeSum = DayAgeSum + 366; 
			} 
			// 平年全年365天 
			else
			{
				DayAgeSum = DayAgeSum + 365;
			}
			// 计算下一年 
			tempYear++;
		}
		
		// 定位到去年 
		Year = tempYear - 1;  // 年 
		Month = Month;		//	月 
		Day = Day; 			// 日
		// 出生月份比当前月份小，够一年，看看加366还是加365 
		if(Month < TodayMonth)
		{
			// printf("出生月份小于当前月份\n");
			// 判断是否为2月之后的月份出生 
			if (Month <= 2) 
			{
				// 判断去年是否为闰年 
				if ((Year%4 == 0&& Year%100 != 0)||(Year%400 == 0))
				{
					// printf("%d 是闰年 \n" , Year);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					// 出生的当前月--》当天的这个月 
					for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
					{
						// 对同一年操作 
						if (TodayYear == Year)
						{
							DayAgeSum = DayAgeSum + RunYear[tempMonth];
						}
						// 对不同年操作 
						else{
							DayAgeSum = DayAgeSum + PingYear[tempMonth];
						}
						
					} 
				} 
				// 去年不为闰年 
				else 
				{
					// 判断今年是否为闰年 
					if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
					{
						// printf("%d 是闰年 \n" , TodayYear);
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						// 出生的当前月--》当天的这个月 
						for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
						{
							// 最后减去的Day是出生那天的日
							DayAgeSum = DayAgeSum + RunYear[tempMonth];
						} 
					}
					else
					{
						//另一种计算方法 
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						// 出生的当前月--》当天的这个月 
						for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
						{
							// 最后减去的Day是出生那天的日
							DayAgeSum = DayAgeSum + PingYear[tempMonth];
						}
					}
					
				} 
			}
			// month > 2,在之前出生 
			else
			{
				// 今年的月份一定大于3 
				// 判断今年是否为闰年 
				if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
				{
					// printf("%d 是闰年 \n" , TodayYear);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					// 出生的当前月--》当天的这个月 
					for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
					{
						// 最后减去的Day是出生那天的日, 今年是闰年 
						DayAgeSum = DayAgeSum + PingYear[tempMonth];
					} 
				} 
				else
				{
					//另一种计算方法 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
					// 出生的当前月--》当天的这个月 
					for(int tempMonth = Month; tempMonth < TodayMonth; tempMonth++)
					{
						// 最后减去的Day是出生那天的日，今年是平年 
						DayAgeSum = DayAgeSum + PingYear[tempMonth];
					}
				} 	
			}
			// 加上今天的日, 把出生那天减掉 
			DayAgeSum = abs(DayAgeSum + TodayDay - Day);
		}
		// 出生月份等于当前月份 
		else if(Month == TodayMonth)
		{
			// printf("出生月份与当前月份相等");
			// Month<= 2时，只需要考虑去年是不是闰年 
			if(Month <= 2)
			{
				// 判断去年是否为闰年 
				if ((Year%4 == 0&& Year%100 != 0)||(Year%400 == 0))
				{
					// printf("%d 是闰年 \n" , Year);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
				} 
				else
				{
					//另一种计算方法 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
				} 
			 } 
			 // Month > 2时只需要考虑今年是不是闰年 
			 else
			 {
			 	// 判断去年是否为闰年 
				if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
				{
					// printf("%d 是闰年 \n" , TodayYear);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
				} 
				else
				{
					//另一种计算方法 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
				} 
			 } 
			// 减掉出生的那天的日加上今天的日 
			DayAgeSum = abs(DayAgeSum + TodayDay - Day);
		} 
		// 出生月份大于当前月份 
		else if(Month > TodayMonth)
		{
			// printf("出生月份大于当前月份\n");
			// 判断今天是否超过2月，超过2月得考虑是否是闰年 
			if(TodayMonth > 2)
			{
				// 判断今年是否为闰年 
				if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
				{
					// printf("%d 是闰年 \n" , TodayYear);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					
					int tempMonth;
					// 应该从下一个月开始 减  
					for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
					{
						// 今年是闰年，减掉的数据应该是闰年的 
						DayAgeSum = abs(DayAgeSum - RunYear[tempMonth]);
					}
					// 我计算的方式是将日期延长比如：出生在2018年10月16，今天是2019年3月15
					//  假设今天是2019年10月10日，那计算就简单了，一年时间-（10月16日到3月15的天数） 
					// 如果今天是3月15日 
					// 出生是10月16日 
					// 今年是闰年，减掉的数据应该是闰年的 
					DayAgeSum = abs(DayAgeSum - (RunYear[TodayMonth] - TodayDay) - Day);
				} 
				else
				{
					//另一种计算方法 
					DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
					
					int tempMonth;
					// 应该从下一个月开始 减  
					for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
					{
						// 今年是平年，减掉的数据应该是平年的 
						DayAgeSum = abs(DayAgeSum - PingYear[tempMonth]);
					}
					// 我计算的方式是将日期延长比如：出生在2018年10月16，今天是2019年3月15
					//  假设今天是2019年10月10日，那计算就简单了，一年时间-（10月16日到3月15的天数） 
					// 如果今天是3月15日 
					// 出生是10月16日 
					// 今年是平年，减掉的数据应该是平年的 
					DayAgeSum = abs(DayAgeSum - (PingYear[TodayMonth] - TodayDay) - Day);
				} 
				
			} 
			// 说明在2月前出生，得考虑去年的是否是闰年 
			if(Month <= 2)
			{
				// 判断去年是否为闰年 
				if ((Year%4 == 0&& Year%100 != 0)||(Year%400 == 0))
				{
					// printf("%d 是闰年 \n" , Year);
					DayAgeSum = DayAgeSum + 366*(TodayYear - Year);
					int tempMonth;
					// 应该从下一个月开始 减  
					for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
					{
						// 去年是润年，减掉的数据应该是平年的 
						DayAgeSum = abs(DayAgeSum - PingYear[tempMonth]);
					}
					// 我计算的方式是将日期延长比如：出生在2018年10月16，今天是2019年3月15
					//  假设今天是2019年10月10日，那计算就简单了，一年时间-（10月16日到3月15的天数） 
					// 如果今天是3月15日 
					// 出生是10月16日 
					
					DayAgeSum = abs(DayAgeSum - (PingYear[TodayMonth] - TodayDay) - Day);
				} 
				else
				{
					// 判断今年是否为闰年 
					if ((TodayYear%4 == 0&& TodayYear%100 != 0)||(TodayYear%400 == 0))
					{
						// printf("%d 是闰年 \n" , TodayYear);
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						int tempMonth;
						// 应该从下一个月开始 减  
						for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
						{
							// 今年为闰年，减去的是闰年的数据 
							DayAgeSum = abs(DayAgeSum - RunYear[tempMonth]);
						}
						// 我计算的方式是将日期延长比如：出生在2018年10月16，今天是2019年3月15
						//  假设今天是2019年10月10日，那计算就简单了，一年时间-（10月16日到3月15的天数） 
						// 如果今天是3月15日 
						// 出生是10月16日 
						// 今年为闰年，减去的是闰年的数据
						DayAgeSum = abs(DayAgeSum - (RunYear[TodayMonth] - TodayDay) - Day);
					} 
					else
					{
						//另一种计算方法 
						DayAgeSum = DayAgeSum + 365*(TodayYear - Year);
						int tempMonth;
						// 应该从下一个月开始 减  
						for(tempMonth = TodayMonth + 1; tempMonth < Month; tempMonth++)
						{
							// 今年为平年，减去的是平年的数据
							DayAgeSum = abs(DayAgeSum - PingYear[tempMonth]);
						}
						// 我计算的方式是将日期延长比如：出生在2018年10月16，今天是2019年3月15
						//  假设今天是2019年10月10日，那计算就简单了，一年时间-（10月16日到3月15的天数） 
						// 如果今天是3月15日 
						// 出生是10月16日 
						// 今年为平年，减去的是平年的数据
						DayAgeSum = abs(DayAgeSum - (PingYear[TodayMonth] - TodayDay) - Day);
					}
				} 
			}	
		} 
		printf("%d\n", DayAgeSum);
		printf("\nDo you want to continue? 【0】No 【1】Yes \n");
		printf("Your Answer is ="); 
		// 修改程序是否执行的标志 
		scanf("%d", &ContinueFlag); 
	}
	printf("Thanks for your use. See you~");
	return 0;
}
