#include <cstdio>
#include <cstring>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <iostream>
using namespace std;
const int maxn = 130;
bool mp[maxn][maxn];
int some[maxn][maxn], none[maxn][maxn], all[maxn][maxn];
int n, m, ans;
void dfs(int d, int an, int sn, int nn)
{
	if(!sn && !nn) ans = max(ans, an);
	int u = some[d][0];
	for(int i = 0; i < sn; ++i)
	{
		int v = some[d][i];
		if(mp[u][v]) continue;
		for(int j = 0; j < an; ++j)
		all[d+1][j] = all[d][j];
		all[d+1][an] = v;
		int tsn = 0, tnn = 0;
		for(int j = 0; j < sn; ++j)
		if(mp[v][some[d][j]])
		some[d+1][tsn++] = some[d][j];
		for(int j = 0; j < nn; ++j)
		if(mp[v][none[d][j]])
		none[d+1][tnn++] = none[d][j];
		dfs(d+1, an+1, tsn, tnn);
		some[d][i] = 0, none[d][nn++] = v;
	}
}
int work()
{
	ans = 0;
	for(int i = 0; i < n; ++i) some[1][i] = i+1;
	dfs(1, 0, n, 0);
	return ans;
}
int main()
{
	while(~scanf("%d", &n) && n)
	{
		srand((unsigned)time(0));
		for(int i = 1; i <= n; ++i)
		for(int j = 1; j <= n; ++j)
		{
			int x; //scanf("%d", &x);
			if( i == j)
			{
				mp[i][j] = 0;
			}
			else
			{
				x = rand()%2;
				mp[i][j] = mp[j][i] = x;	
			}
		}
		for(int i = 1; i <= n; ++i)
		for(int j = 1; j <= n; ++j)
		{
			cout << mp[i][j] << " ";
			if(j == n)
			{
				cout << endl;
			}
		}
		printf("%d\n", work());
	}
	return 0;
}

