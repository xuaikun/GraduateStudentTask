#include <iostream>
#include <cstring>
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <set>
#include <stack>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#define N 1010
#define M 100010
#define LL __int64
#define inf 2e9
#define lson l,mid,ans<<1
#define rson mid+1,r,ans<<1|1
#define getMid (l+r)>>1
#define movel ans<<1
#define mover ans<<1|1
using namespace std;
const LL mod = 1000000007;
const double eps = 1e-8;
vector<int> mapp[N];
int mp[N][N];
int n, m, ans, s;
int temp[N];
void init() {
    for (int i = 1; i <= n; i++) {
        mapp[i].clear();
    }
    ans = 0;
    memset(mp, 0, sizeof(mp));
}
void add(int a, int b) {
    if (a > b) swap(a, b);//??????????
    mapp[a].push_back(b);
    mp[a][b] = mp[b][a] = 1;
}
void dfs(int u, int size) {
    if (size == s) {
        ans++;
        return;
    }
    bool flag;
    for (int i = 0; i < mapp[u].size(); i++) {//??????????
        int v = mapp[u][i];
        flag = true;
        for (int i = 1; i <= size; i++) {//?????????????????????
            if (!mp[v][temp[i]]) {
                flag = false;
                break;
            }
        }
        if (flag) {
            temp[size + 1] = v;
            dfs(v, size+1);
        }
    }
}
int main() {
    cin.sync_with_stdio(false);
    int T, a, b;
    cin >> T;
    while (T--) {
        cin >> n >> m >> s;
        init();
        for (int i = 0; i < m; i++) {
            cin >> a >> b;
            add(a, b);
        }
        for (int i = 1; i <= n; i++) {
            temp[1] = i;//????????
            dfs(i, 1);
        }
        cout << ans << endl;
    }
    return 0;
}
