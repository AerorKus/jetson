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
        jetsontx2GPIO STEP;
        jetsontx2GPIO DIRECTION;
        float velocity;
        float distance;//inches
        float linear_ratio;//how long linear compared to Sramp
};


void MotorInit(jetsontx2GPIO _ENA, jetsontx2GPIO _STEP, jetsontx2GPIO _DIR) {

        cout << "exporting pins" << endl;
        // Make the button and led available in user space
        gpioExport(_ENA);
        gpioExport(_DIR);
        gpioExport(_STEP);
        gpioSetDirection(_ENA, outputPin);
        gpioSetDirection(_DIR, outputPin);
        gpioSetDirection(_STEP, outputPin);

}


void signalHandler(int signum) {
        cout << "Disabling motors\n";

        // cleanup and close up stuff here
        // terminate program
        jetsonTX2GPIONumber enaX = gpio255;     // Output - xaxis motors
        jetsonTX2GPIONumber stepX = gpio428; // Output
        jetsonTX2GPIONumber dirX = gpio429; // Output
        jetsonTX2GPIONumber enaEE = gpio427;     // Output - EE motors
        jetsonTX2GPIONumber stepEE = gpio298; // Output
        jetsonTX2GPIONumber dirEE = gpio398; // Output
	jetsonTX2GPIONumber enaY = gpio396;     // Output - yaxis motors
        jetsonTX2GPIONumber stepY = gpio397; // Output
        jetsonTX2GPIONumber dirY = gpio466; // Output

        gpioSetValue(enaX, low);
        gpioSetValue(dirX, low);
        gpioSetValue(stepX, low);
        gpioSetValue(enaEE, low);
        gpioSetValue(dirEE, low);
        gpioSetValue(stepEE, low);
        gpioSetValue(enaY, low);
        gpioSetValue(dirY, low);
        gpioSetValue(stepY, low);


        gpioUnexport(enaX);     // unexport the ENA
        gpioUnexport(dirX);      // unexport the DIR
        gpioUnexport(stepX);      // unexport the STEP
        gpioUnexport(enaEE);     // unexport the ENA
        gpioUnexport(dirEE);      // unexport the DIR
        gpioUnexport(stepEE);  	// unexport the STEP	       
       	gpioUnexport(enaEE);	// unexport the ENA
        gpioUnexport(dirEE);      // unexport the DIR
        gpioUnexport(stepEE);  //unexport the STEP


        exit(signum);
}


int Sramp(int up_down,float d, float v_CV,float linear_ratio,jetsontx2GPIO STEP ){
	
	float v=0;
	float delay,ta,a,time_total,curve_ratio,seg_time;
	float time;
	float sarray[2000];

	time_total=(d)/v_CV;
       //cout<<"total time = "<<time_total<<endl;	
	curve_ratio=(1-linear_ratio)/4.0;
	//cout<<"curve ratio = "<<curve_ratio<<endl;

	ta=2.0*curve_ratio*time_total;
	a=2*v_CV/ta;
	//cout<<"acceleration = "<<a<<endl;
	//cout<<"ta = "<<ta<<endl;

	seg_time=ta/2;
	//cout<<"seg_time = "<<seg_time<<endl;

	time=seg_time/9;
	//cout<<"time = "<<time<<endl;
	int s=0;
	
	while(time<ta){
		if(time<=seg_time){
			v=a*a/(2*v_CV)*(time*time);
			}
		else if(time>seg_time&&time<=ta){
			v=v_CV-a*a/(2*v_CV)*(ta-time)*(ta-time);
			}
	
			delay=0.00403/v;
			sarray[s]=delay;
			time=time+delay;
			s++;
		}

	time=0;
	if(up_down){//up is 1, down is 0
	for(int i=0;i<s;i++){	
		gpioSetValue(STEP,on);                
		usleep(1000000*sarray[i]/2);
		gpioSetValue(STEP,off);
		usleep(1000000*sarray[i]/2);
		//cout<<"up "<<1000000*sarray[i]/2<<endl;
	}
	}
	else{
	for(int i=s-1;i>0;i--){
		gpioSetValue(STEP,on);
		usleep(1000000*sarray[i]/2);
		gpioSetValue(STEP,off);
		usleep(1000000*sarray[i]/2);
		//cout<<"down "<<1000000*sarray[i]/2<<endl;
	}
	}
	return s;
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

int main(int argc, char *argv[]){

//-----clean gpio-----------
signal(SIGINT, signalHandler);

//--------gpio pin---------
jetsonTX2GPIONumber enaX = gpio255;     // Output - xaxis motors
jetsonTX2GPIONumber stepX = gpio428; // Output
jetsonTX2GPIONumber dirX = gpio429; // Output
jetsonTX2GPIONumber enaEE = gpio427;     // Output - EE motors
jetsonTX2GPIONumber stepEE = gpio298; // Output
jetsonTX2GPIONumber dirEE = gpio398; // Output
jetsonTX2GPIONumber enaY = gpio396;     // Output - yaxis motors
jetsonTX2GPIONumber stepY = gpio397; // Output
jetsonTX2GPIONumber dirY = gpio466; // Output

//----export and set direction of gpio -----
MotorInit(enaX, stepX, dirX);
MotorInit(enaEE, stepEE, dirEE);
MotorInit(enaY, stepY, dirY);

//--------Sramp parameters---------
float d = 10; //predetermined
float linear_ratio = 0.75; //length of linear region compared to ramp
float step = 1600 / 6.447; //count per linear inch
float step_total = step * d; //total counts to travel
double linear_velocity; 
double linear_delay;

//--------ADC initiate----------
int i2cfile = init_i2c(); //pass file descriptor
float data; //volts
float VPS = 6.144 / 2048; //FSR per 2^11 (11 bits, 12th bit is polarity +/-)
float vPV = 2 / 1.2; //velocity per volt MUST CHANGE LATER
int16_t val; //voltage in binary
uint8_t read_buf[2]; //returned values
double pv, inc;

//--------fifo file------------
const char * sizefifo = "/tmp/size"; //file locations
const char * speedfifo = "/tmp/speed";
const char * startfifo = "/tmp/start";
	
float CVspeed = 0; //initiate data
float CVsize = 0;
bool CVstart = 0;

int fdsize = open(sizefifo, O_RDONLY); //open fifo file
int fdspeed = open(speedfifo, O_RDONLY);
int fdstart = open(startfifo, O_RDONLY);
//---------pid parameters---------
PID xpid(0.00001,100,-100,1,0.01,4);// dt, max, min, Kp, Kd, Ki
				    // min/max saturates overshoot

//--------start code-------------

//--------y motor----------------

//--------X/EE motor and Solenoids------------
read(fdspeed, &CVspeed, sizeof(CVspeed));//CV velocity;
cout<<"Speed = "<<CVspeed<<endl;

int up_down=1; //accelerate

gpioSetValue(enaX,on); // turn on motor
gpioSetValue(dirX,on); //on is CCW, off is CW looking at the motor

int ramp_step = Sramp(up_down, d, CVspeed, linear_ratio, stepX); //amount of steps taken ramp up
int linear_step = step_total - 2*ramp_step; //how many steps to reach desired distance

//-----------------closed loop-------------
int current_step=0;
float avg_voltage=0;
double setpoint = CVspeed;
cout<<"CV speed = "<<CVspeed<<endl;
linear_velocity=CVspeed;
while(current_step<linear_step){
	for(int DSP=0;DSP<5;DSP++){
		read(i2cfile,read_buf,2);
		val = read_buf[0] <<4 | read_buf[1]>>4;
		data=val*VPS;//steps x volts per steps = volts
		avg_voltage = avg_voltage + data;
		if(DSP==4) avg_voltage=avg_voltage/5;
		
	}
	cout<<"avg_voltage "<<avg_voltage<<endl;	
	pv = (6 - avg_voltage)*vPV;//process variable, volts x velocity per volts
	cout<<"pv "<<pv<<endl;
	inc = xpid.calculate(setpoint , pv); 
	cout<<"inc "<<inc<<endl;
	linear_velocity = linear_velocity + inc;
	linear_delay=0.00403/linear_velocity;
	cout<<"linear velocity "<<linear_velocity<<"\n"<<endl;

	gpioSetValue(stepX,on);
	usleep(1000000*linear_delay/2);
	gpioSetValue(stepX,off);
	usleep(1000000*linear_delay/2);
	current_step++;
}

//-----------decelerate------------
up_down=0;//decelerate
cout<<"CVspeed = "<<CVspeed<<endl;
int ramp2_step = Sramp(up_down, d, CVspeed, linear_ratio, stepX);
int actual_step= ramp_step + linear_step + ramp2_step;//reference for actual distance traveled
float actual_distance = actual_step/step;
cout<<"actual steps = "<<actual_step<<endl;
cout<<"actual distance = "<<actual_distance<<endl;
//---------------------------------------------
signal(SIGINT, signalHandler);


	return 0;
}

	
