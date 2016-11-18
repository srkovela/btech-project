#include <bits/stdc++.h>
using namespace std;
int main(int argc, char const *argv[])
{
	
	int arr[][10]={{0,1,1,1,0,0,0,0,0,0},{0,0,1,1,1,0,0,0,0,0},
				{0,0,0,1,1,1,0,0,0,0},{0,0,0,0,1,1,1,0,0,0},
				{0,0,0,0,0,1,1,1,0,0},{0,0,0,0,0,0,1,1,1,0},
				{0,0,0,0,0,0,0,1,1,1},{0,0,0,0,0,0,0,0,1,1},
				{0,0,0,0,0,0,0,0,0,1},{0,0,0,0,0,0,0,0,0,0}
			};
	int loopid = 0;

	// for(int i=0;i<10;i++){
	// 	for(int j=0;j<10;j++){
	// 		if(arr[i][j] == 1){
	// 			string lane = "\""+to_string(i+1)+"to"+to_string(j+1)+"_0\"";
	// 			cout<<"<e1Detector id="<<"\"loop"+to_string(loopid++)+"\" lane="<<lane<<" pos=\"75\" freq=\"30\" file=\"sim.out\" friendlyPos=\"x\"/>"<<endl;
	// 		}
	// 	}
	// }

	int n = 10;
	int dp[10];

	vector<vector<int> > d(n,vector<int>(n,1e8));
	priority_queue<pair<int,int>,vector<pair<int,int> > > pq;
	vector<int> srclist,deslist;
	int routeid = 0;
	for(int i=0;i<10;i++){ //Djikstra from each source
		pq.push(make_pair(0,i));
		d[i][i] = 0;
		memset(dp,-1,sizeof dp);
		while(!pq.empty()){
			int u = pq.top().second;
			pq.pop();
			for(int j=0;j<n;j++){
				if(arr[u][j] == 1){
					if(arr[u][j]+d[i][u] < d[i][j]){
						d[i][j] = arr[u][j]+d[i][u];
						dp[j] = u;
						pq.push(make_pair(-1*d[i][j],j));
					}
				}
			}
		}
		//cout<<i<<": \n";
		for(int j=0;j<10;j++){
			//cout<<"dist from "<<i<<" to "<<j<<" is: "<<d[i][j]<<endl;
			stack<pair<int,int> > st;
			int x = j;
			while(dp[x] != -1 && x != i){
				st.push(make_pair(dp[x],x));
				x = dp[x];
			}
			string route = "";
			while(!st.empty()){
				route += to_string(st.top().first+1)+"to"+to_string(st.top().second+1)+" ";
				st.pop();
			}
			//cout<<endl;
			//cout<<dp[j]<<" \n"[i == n-1];
			if(route == "")continue;
			string rid = "r" + to_string(i+1) + "to"+to_string(j+1);
			cout<<"<route id=\""<<rid<<"\" edges=\""<<route<<"\" />"<<endl;
		}
	}
	return 0;
}