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
        jetsontx2GPIO encoder;
	int counter;
	bool enable;
};

void signalHandler(int signum) {
        cout << "Disabling motors\n";

        jetsonTX2GPIONumber encoder = gpio296;     // Output - xaxis motors
        gpioUnexport(encoder);     // unexport the ENA
        exit(signum);
}


void MotorInit(jetsontx2GPIO encoder) {

        cout << "exporting pins" << endl;
        gpioExport(encoder);
        gpioSetDirection(encoder, inputPin);

}

void *encoder(void* threadarg){
	struct thread_data* my_data = (struct thread_data*) threadarg;
	
	jetsontx2GPIO encoder = my_data->encoder;
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
//cout<<"count = "<<counter<<endl;
}

pthread_exit(NULL);	
}

int main(){

signal(SIGINT, signalHandler);
jetsonTX2GPIONumber encoderY1 = gpio296;
MotorInit(encoderY1);

struct thread_data td;
td.encoder = encoderY1;
td.enable=1;

pthread_t tid;
pthread_attr_t attr;
pthread_attr_init(&attr);

pthread_create(&tid, &attr, encoder, &td);
for(int i=0; i<10 ; i++){
cout<<"run = "<<i<<endl;
cout<<"count = "<<td.counter<<endl;
usleep(1000000);
}
td.enable=0;
pthread_join(tid, NULL);

signal(SIGINT, signalHandler);
return 0;

}
