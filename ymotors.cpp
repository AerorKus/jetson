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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <inttypes.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "jetsontx2GPIO.h"
#include "pid.h"
using namespace std;

struct thread_data {
        jetsontx2GPIO GPIO;
	int counter;
	bool enable;
};

struct calc_Sramp_data {
	float delay_array[2000];
	int step;
};

void signalHandler(int signum) {
        cout << "Disabling motors\n";

	jetsonTX2GPIONumber encoderL = gpio396; //encoder left y motor
	jetsonTX2GPIONumber dirYL = gpio466; // dir left y motor
	jetsonTX2GPIONumber stepYL = gpio397; // step left y motor	
	jetsonTX2GPIONumber encoderR = gpio255; //encoder right y motor
	jetsonTX2GPIONumber dirYR = gpio298; // dir right y motor
	jetsonTX2GPIONumber stepYR = gpio389; // step right y motor

	gpioSetValue(dirYL, low);
        gpioSetValue(stepYL, high);
        gpioSetValue(dirYR, low);
        gpioSetValue(stepYR, high);

	gpioUnexport(encoderL);
        gpioUnexport(dirYL);
        gpioUnexport(stepYL);
        gpioUnexport(encoderR);
        gpioUnexport(dirYR);
        gpioUnexport(stepYR);

	exit(signum);
}


void MotorInit(
jetsontx2GPIO encoderL,
jetsontx2GPIO dirYL,
jetsontx2GPIO stepYL,
jetsontx2GPIO encoderR,
jetsontx2GPIO dirYR,
jetsontx2GPIO stepYR
){
	cout << "exporting pins" << endl;

        // Make the button and led available in user space
	gpioExport(encoderL);
        gpioExport(dirYL);
        gpioExport(stepYL);
        gpioExport(encoderR);
        gpioExport(dirYR);
	gpioExport(stepYR);

	//set to input or output
        gpioSetDirection(encoderL, inputPin);
        gpioSetDirection(dirYL, outputPin);
	gpioSetDirection(stepYL, outputPin);
	gpioSetDirection(encoderR, inputPin);
	gpioSetDirection(dirYR, outputPin);
	gpioSetDirection(stepYR, outputPin);
}

void *encoder(void* threadarg){
	struct thread_data* my_data = (struct thread_data*) threadarg;
	
	jetsontx2GPIO encoder = my_data->GPIO;
	int counter=0;
	int enable=1; 
	unsigned int value= 0;

	while(enable){
	enable = my_data->enable;	
	while(value==0){
        	gpioGetValue(encoder, &value);
		if(my_data->enable==0) break;
		usleep(100); //max frequency of 10kHz from encoder
	}
	if(my_data->enable==1) 
	counter++;
	my_data->counter=counter;
	while(value==1){
	        gpioGetValue(encoder,&value);
		if(my_data->enable==0) break;
		usleep(100);
}
cout<<"count = "<<counter<<endl;
}

pthread_exit(NULL);	
}


struct calc_Sramp_data Sramp_function(float d, float v_CV,float linear_ratio){

	struct calc_Sramp_data x;	
	
	float v=0;
	float delay,ta,a,time_total,curve_ratio,seg_time;
	float time;

	time_total=d/v_CV;
        cout<<"total time = "<<time_total<<endl;
	curve_ratio=(1-linear_ratio)/4.0;
	cout<<"curve ratio = "<<curve_ratio<<endl;

	ta=2.0*curve_ratio*time_total;
	a=2*v_CV/ta;
	cout<<"acceleration = "<<a<<endl;
	cout<<"ta = "<<ta<<endl;

	seg_time=ta/2;
	cout<<"seg_time = "<<seg_time<<endl;

	time=seg_time/9;
	cout<<"time = "<<time<<endl;
	int s=0;
	
	while(time<ta){
		if(time<=seg_time){
			v=a*a/(2*v_CV)*(time*time);
			}
		else if(time>seg_time&&time<=ta){
			v=v_CV-a*a/(2*v_CV)*(ta-time)*(ta-time);
			}

			delay=0.005625/v;
			x.delay_array[s]=delay;
			time=time+delay;
			s++;
		}
	x.step=s;

return x;
}



int main(){

//---------initiate gpio-----------
signal(SIGINT, signalHandler);

jetsonTX2GPIONumber encoderL = gpio396; //encoder left y motor
jetsonTX2GPIONumber dirYL = gpio466; // dir left y motor
jetsonTX2GPIONumber stepYL = gpio397; // step left y motor      
jetsonTX2GPIONumber encoderR = gpio255; //encoder right y motor
jetsonTX2GPIONumber dirYR = gpio298; // dir right y motor
jetsonTX2GPIONumber stepYR = gpio389; // step right y motor

MotorInit(encoderL,dirYL,stepYL,encoderR,dirYR,stepYR);

//----------thread--------------
struct thread_data EDL, EDR; //encoder data left/right
EDL.GPIO = encoderL;
EDR.GPIO = encoderR;

EDL.counter = 0;
EDR.counter = 0;

EDL.enable = 1;
EDR.enable = 1;

pthread_t YL, YR; // left/right y encoder
pthread_attr_t attr;
pthread_attr_init(&attr);

//-----------code--------------
pthread_create(&YL, &attr, encoder, &EDL);
pthread_create(&YR, &attr, encoder, &EDR);

gpioSetValue(dirYL,off); //CW
gpioSetValue(dirYR,on); //CCW

float d = 24;
float v_CV = 4.68;
float linear_ratio = 0.75;
int current_step;
float step = 1600/9;
int total_step=step*d;
float linear_delay = 0.005625/v_CV;
struct calc_Sramp_data x =  Sramp_function(d, v_CV, linear_ratio);

int linear_step = total_step - 2*x.step;
//------------acceleration-----------
current_step=0;
while(current_step<x.step){ //correct place wrong time
	if(EDL.counter == EDR.counter){
		gpioSetValue(stepYL,on);
		gpioSetValue(stepYR,on);
	        usleep(1000000*x.delay_array[current_step]/2);
        	gpioSetValue(stepYL,off);
		gpioSetValue(stepYR,off);
		usleep(1000000*x.delay_array[current_step]/2);
		current_step++;
	}
	else if(EDL.counter > EDR.counter){
                gpioSetValue(stepYR,on);
                usleep(1000000*x.delay_array[current_step]/2);
                gpioSetValue(stepYR,off);
                usleep(1000000*x.delay_array[current_step]/2);
	}
        else if(EDL.counter < EDR.counter){
                gpioSetValue(stepYL,on);
                usleep(1000000*x.delay_array[current_step]/2);
                gpioSetValue(stepYL,off);
                usleep(1000000*x.delay_array[current_step]/2);
        }
	else;

}
//----------linear---------------
current_step=0;

while(current_step<linear_step){
        if(EDL.counter == EDR.counter){
                gpioSetValue(stepYL,on);
                gpioSetValue(stepYR,on);
                usleep(1000000*linear_delay/2);
                gpioSetValue(stepYL,off);
                gpioSetValue(stepYR,off);
                usleep(1000000*linear_delay/2);
                current_step++;
        }
        else if(EDL.counter > EDR.counter){
                gpioSetValue(stepYR,on);
                usleep(1000000*linear_delay/2);
                gpioSetValue(stepYR,off);
                usleep(1000000*linear_delay/2);
        }
        else if(EDL.counter < EDR.counter){
                gpioSetValue(stepYL,on);
                usleep(1000000*linear_delay/2);
                gpioSetValue(stepYL,off);
                usleep(1000000*linear_delay/2);
        }
        else;

}




//---------deceleration------------
current_step=0;
while(current_step<x.step){ //correct place wrong time
        if(EDL.counter == EDR.counter){
                gpioSetValue(stepYL,on);
                gpioSetValue(stepYR,on);
                usleep(1000000*x.delay_array[x.step-1-current_step]/2);
                gpioSetValue(stepYL,off);
                gpioSetValue(stepYR,off);
                usleep(1000000*x.delay_array[x.step-1-current_step]/2);
                current_step++;
        }
        else if(EDL.counter > EDR.counter){
                gpioSetValue(stepYR,on);
                usleep(1000000*x.delay_array[x.step-1-current_step]/2);
                gpioSetValue(stepYR,off);
                usleep(1000000*x.delay_array[x.step-1-current_step]/2);
        }
        else if(EDL.counter < EDR.counter){
                gpioSetValue(stepYL,on);
                usleep(1000000*x.delay_array[x.step-1-current_step]/2);
                gpioSetValue(stepYL,off);
                usleep(1000000*x.delay_array[x.step-1-current_step]/2);
        }
        else;

}

EDL.enable=0;
EDR.enable=0;
pthread_join(YL, NULL);
pthread_join(YR, NULL);

//create another circuit push button circuit
//attach one motor
//if click one button, one motor should run
//if click other button, both run
//test both motors

signal(SIGINT, signalHandler);
return 0;

}

