#include <iostream>
using namespace std;

int main(void)
{
	int N;              //define the number of nodes
	cin >> N;           //input the the number of nodes
	int Es = 10;       // 假设 节点的电量 为 10KJ
	int V = 5 ; 		   // 定义MC的移动速度 m/s
	int qm = 5;       // MC的移动功耗     J/m
	int qcn = 5;     //能量传输率   W
	int Em = 40;	 //假设 MC的总能量为 26KJ
	float Pi = 0.01;   // 假设所有传感器功耗为 0.01 W 
	int qc = 5;      // 5W
Step1:				//计算P的总距离D 
	//假设 每个 node 之间的距离为 1KM  则总距离 D = N
	int D = N;         // 计算P的总距离 D Km
	cout << "D = " << D << "m" << endl;
	
Step2: 				// 收集所有传感器功耗Pi，计算Psum和Pmx
	//Psum = P1+P2+……+Pi(1 <= i <= N)
	//Pmx = max(Pi*(qcn-Pi))(1 <= i <= N)
	float Psum = 0;	//计算Psum 
	for(int i = 1; i <= N; i++)
	{
		Psum = Psum + Pi;	
	} 
	cout << "qcn = " << qcn << "W " << endl; 
	cout << "Psum = " << Psum << " W" << endl;
	float  Pmx =  Pi*(qcn - Pi); //	因为假设 所有 传感器功耗一样所以直接用其中一个求即可
	cout << "Pmx = "  << Pmx << endl;
	cout << "V = " << V << "m/s" << endl;
	cout << "(D*Pmx)/(Es*(qcn - Psum) = "  << (D*Pmx)/(Es*(qcn - Psum)) << " m/s" << endl;
	cout << "Em = " << Em << "KJ" << endl;
	cout << "(D*qm + (Psum*Es*qc)/Pmx) = " << (D*qm + (Psum*Es*qc)/Pmx) << "KJ" << endl;
	if(qcn <= Psum || V < (D*Pmx)/(Es*(qcn - Psum)) || Em < (D*qm + (Psum*Es*qc)/Pmx)) 
	{
		cout << "此回路无法调度" << endl; 
		return 0;
	}
	
	double Tl = (D*qcn)/(V*(qcn - Psum));
	double Tu = (Es*qcn)/Pmx;
	
	double W = (Psum/qcn) * Tu + D/V;
	
	cout << " Tl = " << Tl << endl;
	cout << " Tu = " << Tu << endl;
	cout << " W  = " << W  << endl;
	
	cout << "此回路可调度" << endl; 
	return 0;
}
