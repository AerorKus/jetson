#include "pid.h"
#include <iostream>

using namespace std;

int main(){
	
	PID xpid(0.1,100,-100,0.1,0.01,0.5);
	
	double encoder=20;
	for (int i=0; i<100; i++){
	      	double inc = xpid.calculate(0, encoder);
		cout<<"encoder="<<encoder<<" inc="<<inc<<endl;
		encoder += inc;
	}

return 0;
}
