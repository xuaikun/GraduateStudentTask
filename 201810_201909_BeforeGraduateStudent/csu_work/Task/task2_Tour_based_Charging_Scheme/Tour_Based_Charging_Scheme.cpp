#include <iostream>
using namespace std;

int main(void)
{
	int N;              //define the number of nodes
	cin >> N;           //input the the number of nodes
	int Es = 10;       // ���� �ڵ�ĵ��� Ϊ 10KJ
	int V = 5 ; 		   // ����MC���ƶ��ٶ� m/s
	int qm = 5;       // MC���ƶ�����     J/m
	int qcn = 5;     //����������   W
	int Em = 40;	 //���� MC��������Ϊ 26KJ
	float Pi = 0.01;   // �������д���������Ϊ 0.01 W 
	int qc = 5;      // 5W
Step1:				//����P���ܾ���D 
	//���� ÿ�� node ֮��ľ���Ϊ 1KM  ���ܾ��� D = N
	int D = N;         // ����P���ܾ��� D Km
	cout << "D = " << D << "m" << endl;
	
Step2: 				// �ռ����д���������Pi������Psum��Pmx
	//Psum = P1+P2+����+Pi(1 <= i <= N)
	//Pmx = max(Pi*(qcn-Pi))(1 <= i <= N)
	float Psum = 0;	//����Psum 
	for(int i = 1; i <= N; i++)
	{
		Psum = Psum + Pi;	
	} 
	cout << "qcn = " << qcn << "W " << endl; 
	cout << "Psum = " << Psum << " W" << endl;
	float  Pmx =  Pi*(qcn - Pi); //	��Ϊ���� ���� ����������һ������ֱ��������һ���󼴿�
	cout << "Pmx = "  << Pmx << endl;
	cout << "V = " << V << "m/s" << endl;
	cout << "(D*Pmx)/(Es*(qcn - Psum) = "  << (D*Pmx)/(Es*(qcn - Psum)) << " m/s" << endl;
	cout << "Em = " << Em << "KJ" << endl;
	cout << "(D*qm + (Psum*Es*qc)/Pmx) = " << (D*qm + (Psum*Es*qc)/Pmx) << "KJ" << endl;
	if(qcn <= Psum || V < (D*Pmx)/(Es*(qcn - Psum)) || Em < (D*qm + (Psum*Es*qc)/Pmx)) 
	{
		cout << "�˻�·�޷�����" << endl; 
		return 0;
	}
	
	double Tl = (D*qcn)/(V*(qcn - Psum));
	double Tu = (Es*qcn)/Pmx;
	
	double W = (Psum/qcn) * Tu + D/V;
	
	cout << " Tl = " << Tl << endl;
	cout << " Tu = " << Tu << endl;
	cout << " W  = " << W  << endl;
	
	cout << "�˻�·�ɵ���" << endl; 
	return 0;
}
