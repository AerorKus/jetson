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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <inttypes.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>

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
	    jetsonTX2GPIONumber step = gpio428;     // Output - xaxis motors
            jetsonTX2GPIONumber dir = gpio429; // Output 
	    jetsonTX2GPIONumber ena = gpio255; // Output   
	    
	    gpioSetValue(ena, low);
	    gpioSetValue(dir, low);
	    gpioSetValue(step, low);

	    gpioUnexport(ena);     // unexport the ENA
	    gpioUnexport(dir);      // unexport the DIR
	    gpioUnexport(step);      // unexport the STEP
	    exit(signum);
}

int init_i2c(){
	int file;
	int I2CAddress=0x48;
        const char *filename = "/dev/i2c-1";
	uint8_t config[3];
	uint8_t address[1];

	if ((file = open(filename, O_RDWR)) < 0) {
		/* ERROR HANDLING: you can check errno to see what went wrong */
		perror("Failed to open the i2c bus");
		exit(1);
	}

	if(ioctl(file,I2C_SLAVE,I2CAddress)<0){
		/* ERROR HANDLING; you can check errno to see what went wrong */
		perror("Failed to connect");
		exit(1);
	}

	config[0]=0x01;//points to config register
	config[1]=0xC0;//MSB config register 1100 0000 FSR=6.144, AIN0 to GND
	config[2]=0x83;//LSB config register
	address[0]=0x00;//points to conversion register

	if(write(file, config,3)!=3){//if we dont pass all 3 indexes something went wrong
		perror("Write to config register");
		exit(1);
	}
	if(write(file, address,1)!=1){
                perror("Write to address register");
                exit(1);
        }
	usleep(100);//makes it more realiable

return file;
}


int main(){
	signal(SIGINT, signalHandler);
jetsonTX2GPIONumber enaX = gpio255;     // Output - xaxis motors
jetsonTX2GPIONumber stepX = gpio428; // Output
jetsonTX2GPIONumber dirX = gpio429; // Output
	
MotorInit(enaX, stepX, dirX);

//--------ADC initiate----------
int i2cfile = init_i2c(); //pass file descriptor
float data; //volts
float VPS = 6.144 / 2048; //FSR per 2^11 (11 bits, 12th bit is polarity +/-)
float vPV = 2 / 1.2; //velocity per volt MUST CHANGE LATER
int16_t val; //voltage in binary
uint8_t read_buf[2]; //returned values
float v_FB; //velocity from FV converter

//-------motor parameters----------
	float d=24; //inches (linear)
	float v=0;
	//float v_CV=4.68; // inches per second (linear)(11.89cm/s)
	float v_CV;
	cout<<"v_CV"<<endl;
	cin>>v_CV;
	float delay,ta,a,time_total,curve_ratio,seg_time;
	float time;
	float linear_ratio=0.75;
	float sarray[10000];

	time_total=(d)/v_CV;
       	cout<<"total time ="<<time_total<<endl;	
	curve_ratio=(1-linear_ratio)/4.0;
	cout<<"cure ratio"<<curve_ratio<<"\n"<<endl;

	ta=2.0*curve_ratio*time_total;
	a=2*v_CV/ta;
	cout<<"acceleration"<<a<<"\n"<<endl;
	cout<<"ta="<<ta<<"\n"<<endl;

	seg_time=ta/2;
	cout<<"seg_time="<<seg_time<<endl;

	time=seg_time/9;
	cout<<"time="<<time<<endl;
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
			//cout<<delay<<endl;
		
	}
			
	float linear_delay=0.00403/v_CV;
	time=0;
   	float linear_time = linear_ratio*time_total;
	cout<<s<<endl;
	float avg_data=0;
	for(int i=0;i<s;i++){
		gpioSetValue(stepX,on);                
		usleep(1000000*sarray[i]/2);
		gpioSetValue(stepX,off);
		usleep(1000000*sarray[i]/2);
		cout<<"up "<<1000000*sarray[i]/2<<endl;
		avg_data=0;
		for(int j=0;j<10;j++){
			read(i2cfile,read_buf,2);//read feedback
			val = read_buf[0] << 4 | read_buf[1] >> 4;
			data = val*VPS;//steps x volts per steps = volts
			//cout<<"data = "<<data<<endl;
			avg_data = avg_data + data;
			if(j==9){
				avg_data = avg_data/10;
				cout<<"avg_data = "<<avg_data<<endl;
				v_FB = (6-avg_data)*vPV;//velocity of linear motion not rotational
		}	
			//subtract 6V because differential amp inverts it
	usleep(10);
	}
	cout<<"feedback velocity = "<<v_FB<<"\n"<<endl;
	}
	int current_step=0;
	while(current_step < 1000){
		gpioSetValue(stepX,on);
	        usleep(1000000*linear_delay/2);
	        gpioSetValue(stepX,off);
	        usleep(1000000*linear_delay/2);
		time=time+linear_delay;
		cout<<"linear "<<1000000*linear_delay/2<<endl;
                avg_data=0;
		for(int j=0;j<10;j++){
                        read(i2cfile,read_buf,2);//read feedback
                        val = read_buf[0] << 4 | read_buf[1] >> 4;
                        data = val*VPS;//steps x volts per steps = volts
                        //cout<<"data = "<<data<<endl;
                        avg_data = avg_data + data;
                        if(j==9){
                                avg_data = avg_data/10;
                                cout<<"avg_data = "<<avg_data<<endl;
                                v_FB = (6-avg_data)*vPV;//velocity of linear motion not rotational
                }
                        //subtract 6V because differential amp inverts it
        usleep(10);
        }
        cout<<"feedback velocity = "<<v_FB<<"\n"<<endl;
	current_step++;
	}

	for(int i=s-1;i>0;i--){
		gpioSetValue(stepX,on);
		usleep(1000000*sarray[i]/2);
		gpioSetValue(stepX,off);
		usleep(1000000*sarray[i]/2);
		cout<<"down "<<1000000*sarray[i]/2<<endl;
                avg_data=0;
		for(int j=0;j<10;j++){
                        read(i2cfile,read_buf,2);//read feedback
                        val = read_buf[0] << 4 | read_buf[1] >> 4;
                        data = val*VPS;//steps x volts per steps = volts
                        //cout<<"data = "<<data<<endl;
                        avg_data = avg_data + data;
                        if(j==9){
                                avg_data = avg_data/10;
                                cout<<"avg_data = "<<avg_data<<endl;
                                v_FB = (6-avg_data)*vPV;//velocity of linear motion not rotational
                }
                        //subtract 6V because differential amp inverts it
        usleep(10);
        }
        cout<<"feedback velocity = "<<v_FB<<"\n"<<endl;

	}


return 0;
}
