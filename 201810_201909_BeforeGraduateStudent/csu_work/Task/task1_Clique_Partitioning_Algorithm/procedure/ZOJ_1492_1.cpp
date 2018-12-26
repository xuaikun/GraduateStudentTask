#include <cstdio>
#include <cstring>
#include <iostream>
#include <algorithm>
using namespace std;
const int MAXN = 105;
 
struct MaxClique {
	bool g[MAXN][MAXN];
	int n, dp[MAXN], st[MAXN][MAXN], ans;
	//	dp[i]��ʾ��i����֮������ɵ�����ŵĴ�С��
	// 	st[i][j]��ʾ�㷨�е�i��dfs����Ҫ�ĵ�ļ��ϣ������п��������������֮һ�ĵ� 
 
	void init(int n) {
		this->n = n;
		memset(g, false, sizeof(g));
	}
 
	void addedge(int u, int v, int w) {
		g[u][v] = w;
	}
 
	bool dfs(int sz, int num) {
		if (sz == 0) {
			if (num > ans) {
				ans = num;
				return true;
			}
			return false;
		}
		for (int i = 0; i < sz; i++) {		// �ڵ�num��ļ�����ö��һ����i
			if (sz - i + num <= ans) return false;	// ��֦1
			int u = st[num][i];
			if (dp[u] + num <= ans) return false;	// ��֦2
			int cnt = 0;
			for (int j = i + 1; j < sz; j++) {	// �ڵ�num�������i֮�������i�������ĵ㣬���Ҽ����num+1�㼯��
				if (g[u][st[num][j]]) 
					st[num + 1][cnt++] = st[num][j];
			}
			if (dfs(cnt, num + 1)) return true;
		}
		return false;
	}
 
	int solver() {
		ans = 0;
		memset(dp, 0, sizeof(dp));
		for (int i = n; i >= 1; i--) {
			int cnt = 0;
			for (int j = i + 1; j <= n; j++) {	// ��ʼ����1�㼯��
				if (g[i][j]) st[1][cnt++] = j;
			}
			dfs(cnt, 1);
			dp[i] = ans;
		}
		return ans;
	}
 
}maxclique; 
 
int main() {
	int n;
	while (scanf("%d", &n), n) {
		maxclique.init(n);
		for (int i = 1; i <= n; i++) {
			for (int j = 1; j <= n; j++) {
				int x;
				scanf("%d", &x);
				maxclique.addedge(i, j, x);
			}
		}
		printf("%d\n", maxclique.solver());
	}
	return 0;
}

