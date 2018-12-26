#include <stdio.h> 
#define S(x) (x)*x*2

int main()
{
	int k = 5, j = 2;
	printf("%d,%d\n",S(k+j),S(k-j));
}
