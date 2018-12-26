#include <iostream>
#include <time.h>
#include <stdlib.h>
using namespace std;

#define NUM 10


int main()
{
	srand((unsigned)time(0));
	int A[NUM][NUM];
	for(int i = 0; i < NUM; i++)
	{
		for(int j = 0; j < NUM; j++)
		{
			if(i == j)
			{
				A[i][j] = 0;
			}
			else{
				A[i][j] = rand()%2; 
				A[j][i] = A[i][j]; 
			}
			
		}
	}
	
	for(int i = 0; i < NUM; i++)
	{
		for(int j = 0; j < NUM; j++)
		{ 
		
			cout << A[i][j] << " "; 
			if(j == NUM - 1)
				cout << endl;		
		}
	}
	return 0;
 } 
