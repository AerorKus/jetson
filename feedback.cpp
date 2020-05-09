#include "pid.h"
#include <iostream>

using namespace std;

int main(){
	
	PID xpid(0.1,100,-100,0.1,0.01,0.5);
	
	double encoder=20;
	for (int i=0; i<50; i++){
	      	double inc = xpid.calculate(4.68, encoder);
		cout<<"encoder="<<encoder<<" inc="<<inc<<endl;
		encoder += inc;
	}
/*
	 float SP=4.68;
	 float PV;
	 for (int i=0;i<10;i++){
	 
//	 cout<<"SP = "<<endl;
	// cin>>SP;
	 cout<<"PV = "<<endl;
	 cin>>PV;
	 double inc = xpid.calculate(SP, PV);
//	 cout<<"SP = "<<SP<<" PV = "<<PV<<" inc = "<<inc<<endl;
	cout<<"inc = "<<inc<<endl;
	 }
	 */
return 0;
}
