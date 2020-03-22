#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <pthread.h>
#include <cmath>
#include <csignal>
#include "jetsontx2GPIO.h"
using namespace std;

void MotorInit(jetsontx2GPIO _ENA, jetsontx2GPIO _STEP, jetsontx2GPIO _DIR) {
        cout << "exporting pins" << endl;
        // Make the button and led available in user space
       	 gpioExport(_ENA);
	 gpioExport(_DIR);
         gpioExport(_STEP);
         gpioSetDirection(_ENA, outputPin);      
	 gpioSetDirection(_DIR, outputPin);	
	 gpioSetDirection(_STEP, outputPin);
	 gpioSetValue(_ENA, high);
			 }
			                                                         void signalHandler(int signum) {
	cout << "Disabling motors\n";
	 // cleanup and close up stuff here
	 // terminate program
	    jetsonTX2GPIONumber step = gpio396;     // Output - xaxis motors
            jetsonTX2GPIONumber dir = gpio466; // Output 
	    jetsonTX2GPIONumber ena = gpio397; // Output   
	    jetsonTX2GPIONumber ena120 = gpio398;     // Output - EE motors
 	    jetsonTX2GPIONumber step120 = gpio298; // Output
	    jetsonTX2GPIONumber dir120 = gpio389; // Output                    
	    
	    gpioSetValue(ena, low);
	    gpioSetValue(dir, low);
	    gpioSetValue(step, low);
	    gpioSetValue(ena120, low);
	    gpioSetValue(dir120, low);
	    gpioSetValue(step120, low);

	    gpioUnexport(ena);     // unexport the ENA
	    gpioUnexport(dir);      // unexport the DIR
	    gpioUnexport(step);      // unexport the STEP
                                                                                            gpioUnexport(ena120);     // unexport the ENA
	    gpioUnexport(dir120);      // unexport the DIR
	    gpioUnexport(step120);      // unexport the STEP
	    exit(signum);
										 }
int main(){
	signal(SIGINT, signalHandler);
	jetsonTX2GPIONumber STEP = gpio396;
	jetsonTX2GPIONumber DIR = gpio466;
	jetsonTX2GPIONumber ENA = gpio397;
	
	MotorInit(ENA, STEP, DIR);

	float d=0.5; //inches (linear)
	float v=0;
	float v_CV=4.68; // in per second (linear)
	float delay,ta,a,time_total,curve_ratio,seg_time;
	float time;
	float linear_ratio=0.75;
	float sarray[1000000];

	time_total=(d*100)/v_CV;
	curve_ratio=(1-linear_ratio)/4.0;
	cout<<curve_ratio<<"\n"<<endl;

	ta=2.0*curve_ratio*time_total;
	a=2*v_CV/ta;
	cout<<a<<"\n"<<endl;
	cout<<"ta="<<ta<<"\n"<<endl;

	seg_time=ta/2;
	cout<<"seg_time="<<seg_time<<endl;

	time=seg_time/9;

	int s=0;
	int counta=0, countb=0;

	while(time<ta)
	{
		if(time<=seg_time){
			v=a*a/(2*v_CV)*(time*time);
			//cout<<v<<endl;
			//cout<<va="<<v<<endl;
			counta=counta+1;
		}
		else if(time>seg_time&&time<=ta){
			v=v_CV-a*a/(2*v_CV)*(ta-time)*(ta-time);
			//cout<<v<<endl;
			//cout<<"vb="<<v<<endl;
			countb=countb+1;
		}
	

			delay=0.00403/v;
			sarray[s]=delay;
			time=time+delay;
			s++;
		//	cout<<delay<<endl;
		
	}
			
	float linear_delay=0.00403/v_CV;
	time=0;
   	float linear_time = linear_ratio*time_total;
	cout<<s<<endl;
	for(int i=0;i<s;i++){
		gpioSetValue(STEP,on);                
		usleep(1000000*sarray[i]/2);
		gpioSetValue(STEP,off);
		usleep(1000000*sarray[i]/2);
		cout<<1000000*sarray[i]/2<<endl;
	}
	
	while(time<linear_time){
		gpioSetValue(STEP,on);
	        usleep(1000000*linear_delay/2);
	        gpioSetValue(STEP,off);
	        usleep(1000000*linear_delay/2);
		time=time+linear_delay;
		cout<<1000000*linear_delay/2<<endl;
	}

	for(int i=s;i>0;i--){
		gpioSetValue(STEP,on);
		usleep(1000000*sarray[i]/2);
		gpioSetValue(STEP,off);
		usleep(1000000*sarray[i]/2);
		cout<<1000000*sarray[i]/2<<endl;
	}


return 0;
}
