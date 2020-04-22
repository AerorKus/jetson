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


struct endeffector { 
        jetsontx2GPIO STEP;
	jetsontx2GPIO solenoidIn;
	jetsontx2GPIO solenoidOut;
        float time_total;
	int spray; //0 dont spray, 1 inner, 2 outter	
};

struct encoder{
	jetsontx2GPIO GPIO;
	int counter;
	bool enable;
};

struct calc_Sramp_data {
        float delay_array[2000];
        int step;
};

void *EEMotor(void* threadarg){ 
	struct endeffector* my_data = (struct endeffector*) threadarg;
	
	jetsontx2GPIO STEP = my_data->STEP;
	jetsontx2GPIO solenoidIn = my_data->solenoidIn;
	jetsontx2GPIO solenoidOut = my_data->solenoidOut;
	float time_total = my_data->time_total;
	int spray = my_data->spray;
        if(spray==0){
		gpioSetValue(solenoidIn,off);
		gpioSetValue(solenoidOut,off);
	}
	else if(spray==1){
		gpioSetValue(solenoidIn,on);
		gpioSetValue(solenoidOut,off);
	}
	else if(spray==2){
		gpioSetValue(solenoidIn,off);
		gpioSetValue(solenoidOut,on);
	}
	else;



	float step_total = 534; // 1600/3 = 534 cnts for a 120 degree turn
	float d = 2*3.14159/3; //in radians (2*pi/3)
	
	float linear_velocity = d/time_total; //rad per second
        float linear_delay=0.003927/linear_velocity; // 2*pi/1600 conversion from radians to count
	
	int current_step=0;
        while(current_step<step_total){
                gpioSetValue(STEP,on);
                usleep(1000000*linear_delay/2);
                gpioSetValue(STEP,off);
                usleep(1000000*linear_delay/2);
	        //cout<<"linear "<<1000000*linear_delay/2<<endl;
		current_step++;
	}       
        
  	//stops spraying when end effector reaches the end	
       	gpioSetValue(solenoidIn,off);       
	gpioSetValue(solenoidOut,off);

pthread_exit(NULL);	
}	


void *encoder(void* threadarg){
	struct encoder* my_data = (struct encoder*) threadarg;

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
}

pthread_exit(NULL);
}


void MotorInit(
jetsontx2GPIO encoderL,
jetsontx2GPIO dirYL,
jetsontx2GPIO stepYL,
jetsontx2GPIO encoderR,
jetsontx2GPIO dirYR,
jetsontx2GPIO stepYR,
jetsontx2GPIO dirEE,
jetsontx2GPIO stepEE,
jetsontx2GPIO dirX,
jetsontx2GPIO stepX,
jetsontx2GPIO solenoidOut,
jetsontx2GPIO solenoidIn,
jetsontx2GPIO flow,
jetsontx2GPIO level,
jetsontx2GPIO maintence,
jetsontx2GPIO start_stop,
jetsontx2GPIO OFF,
jetsontx2GPIO emerg_stop,
jetsontx2GPIO disable_Y,
jetsontx2GPIO disable_X_EE,
jetsontx2GPIO extra
)
{

        cout << "exporting pins" << endl;

        // Make the button and led available in user space
	gpioExport(encoderL);
        gpioExport(dirYL);
        gpioExport(stepYL);
        gpioExport(encoderR);
        gpioExport(dirYR);
        gpioExport(stepYR);
        gpioExport(dirEE);
        gpioExport(stepEE);
        gpioExport(dirX);
        gpioExport(stepX);
        gpioExport(solenoidOut);
        gpioExport(solenoidIn);
        gpioExport(flow);
        gpioExport(level);
        gpioExport(maintence);
        gpioExport(start_stop);
        gpioExport(OFF);
        gpioExport(emerg_stop);
        gpioExport(disable_Y);
        gpioExport(disable_X_EE);
        gpioExport(extra);
	
	//set to input or output
        gpioSetDirection(encoderL, inputPin);
        gpioSetDirection(dirYL, outputPin);
	gpioSetDirection(stepYL, outputPin);
	gpioSetDirection(encoderR, inputPin);
	gpioSetDirection(dirYR, outputPin);
	gpioSetDirection(stepYR, outputPin);
	gpioSetDirection(dirEE, outputPin);
	gpioSetDirection(stepEE, outputPin);
	gpioSetDirection(dirX, outputPin);
	gpioSetDirection(stepX, outputPin);
	gpioSetDirection(solenoidOut, outputPin);
	gpioSetDirection(solenoidIn, outputPin);
	gpioSetDirection(flow, inputPin);
	gpioSetDirection(level, inputPin);
	gpioSetDirection(maintence, inputPin);
	gpioSetDirection(start_stop, inputPin);
	gpioSetDirection(OFF, inputPin);
	gpioSetDirection(emerg_stop, inputPin);
	gpioSetDirection(disable_Y, outputPin);
	gpioSetDirection(disable_X_EE, outputPin);
	gpioSetDirection(extra, outputPin);


}


void signalHandler(int signum) {
        cout << "Disabling motors\n";

        // cleanup and close up stuff here
        // terminate program
	//left side
	jetsonTX2GPIONumber encoderL = gpio396; //encoder left y motor
	jetsonTX2GPIONumber dirYL = gpio466; // dir left y motor
	jetsonTX2GPIONumber stepYL = gpio397; // step left y motor	
	jetsonTX2GPIONumber encoderR = gpio255; //encoder right y motor
	jetsonTX2GPIONumber dirYR = gpio429; // dir right y motor
	jetsonTX2GPIONumber stepYR = gpio428; // step right y motor
	jetsonTX2GPIONumber dirEE = gpio427; // dir EE motor
	jetsonTX2GPIONumber stepEE = gpio398; // step EE motor
	jetsonTX2GPIONumber dirX = gpio298; // dir X motor
	jetsonTX2GPIONumber stepX = gpio389; // step X motor
	jetsonTX2GPIONumber solenoidOut = gpio395; // outter two solenoid rings
	jetsonTX2GPIONumber solenoidIn = gpio388; // inner one solenoids ring
	//right side
	jetsonTX2GPIONumber flow = gpio392; // flow from PLC
	jetsonTX2GPIONumber level = gpio296; // level from PLC
	jetsonTX2GPIONumber maintence = gpio481; //use water to clean from PLC
	jetsonTX2GPIONumber start_stop = gpio254; //start_stop program
	jetsonTX2GPIONumber OFF = gpio430; //shut down everything
	jetsonTX2GPIONumber emerg_stop = gpio297; //interrupt motors in place
	jetsonTX2GPIONumber disable_Y = gpio467; //disable both Y motors
	jetsonTX2GPIONumber disable_X_EE = gpio394; //disable both X and EE motors
	jetsonTX2GPIONumber extra = gpio393; // not used
	
	//Setvalue
        gpioSetValue(dirYL, low);
        gpioSetValue(stepYL, high);
        gpioSetValue(dirYR, low);
        gpioSetValue(stepYR, high);
        gpioSetValue(dirEE, low);
        gpioSetValue(stepEE, high);
        gpioSetValue(dirX, low);
        gpioSetValue(stepX, high);
       	gpioSetValue(solenoidOut, low);
        gpioSetValue(solenoidIn, low); 
	gpioSetValue(disable_Y, low);
        gpioSetValue(disable_X_EE, low);
        gpioSetValue(extra, low);

	//Unexport
        gpioUnexport(encoderL);  
        gpioUnexport(dirYL);   
        gpioUnexport(stepYL);     
        gpioUnexport(encoderR);     
        gpioUnexport(dirYR);   
        gpioUnexport(stepYR); 
        gpioUnexport(dirEE);     
	gpioUnexport(stepEE); 
	gpioUnexport(dirX);
        gpioUnexport(stepX);
	gpioUnexport(solenoidOut);
        gpioUnexport(solenoidIn);
	gpioUnexport(flow);	
       	gpioUnexport(level);	
       	gpioUnexport(maintence);	
       	gpioUnexport(start_stop);	
	gpioUnexport(OFF);	
       	gpioUnexport(emerg_stop);	
       	gpioUnexport(disable_Y);	
       	gpioUnexport(disable_X_EE);	
       	gpioUnexport(extra);	

        exit(signum);
}

//used for calculations return in main function
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

//used for to move motors
int Sramp(int up_down,float d, float v_CV,float linear_ratio,jetsontx2GPIO STEP ){
	
	float v=0;
	float delay,ta,a,time_total,curve_ratio,seg_time;
	float time;
	float sarray[2000];

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
		cout<<"up "<<1000000*sarray[i]/2<<endl;
	}
	}
	else{
	for(int i=s-1;i>0;i--){
		gpioSetValue(STEP,on);
		usleep(1000000*sarray[i]/2);
		gpioSetValue(STEP,off);
		usleep(1000000*sarray[i]/2);
		cout<<"down "<<1000000*sarray[i]/2<<endl;
	}
	}
	return s;
}

int linear_pid(int pid_linear, double CVspeed, int ramp_step, int step_total, int i2cfile, jetsontx2GPIO STEP){
	//-------i2c---------
	float data; //volts
	float VPS = 6.144 / 2048; //FSR per 2^11 (11 bits, 12th bit is polarity +/-)
	float vPV = 2 / 1.2; //velocity per volt MUST CHANGE LATER
	int16_t val; //voltage in binary
	uint8_t read_buf[2]; //returned values
	
	//--------closed loop-------
	int current_step=0;
	float avg_voltage=0;
	double setpoint = CVspeed;
	double linear_velocity=CVspeed;
	double linear_delay;
	double pv, inc;
	int linear_step = step_total - 2*ramp_step; //how many steps to reach desired distance
	
	//--------pid parameters---------
        PID xpid(0.00001,100,-100,1,0.01,4);// dt, max, min, Kp, Kd, Ki
                                            // min/max saturates overshoot
	//--------pid----------
	if(pid_linear){
	while(current_step<=linear_step){
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
	        linear_delay=0.005625/linear_velocity;
	        cout<<"linear velocity "<<linear_velocity<<"\n"<<endl;
	
	        gpioSetValue(STEP,on);
	        usleep(1000000*linear_delay/2);
	        gpioSetValue(STEP,off);
	        usleep(1000000*linear_delay/2);
	        current_step++;
	}
	}
	else{  //linear
	linear_delay=0.005625/linear_velocity;
	while(current_step<linear_step){
		gpioSetValue(STEP,on);
	        usleep(1000000*linear_delay/2);
        	gpioSetValue(STEP,off);
	        usleep(1000000*linear_delay/2);
	        current_step++;
	}
	}

	
	
	return linear_step;
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
//left side
jetsonTX2GPIONumber encoderL = gpio396; //encoder left y motor
jetsonTX2GPIONumber dirYL = gpio466; // dir left y motor
jetsonTX2GPIONumber stepYL = gpio397; // step left y motor

jetsonTX2GPIONumber encoderR = gpio255; //encoder right y motor
jetsonTX2GPIONumber dirYR = gpio429; // dir right y motor
jetsonTX2GPIONumber stepYR = gpio428; // step right y motor

jetsonTX2GPIONumber dirEE = gpio427; // dir EE motor
jetsonTX2GPIONumber stepEE = gpio398; // step EE motor
jetsonTX2GPIONumber dirX = gpio298; // dir X motor
jetsonTX2GPIONumber stepX = gpio389; // step X motor

jetsonTX2GPIONumber solenoidOut = gpio395; // outter two solenoid rings
jetsonTX2GPIONumber solenoidIn = gpio388; // inner one solenoids ring

//right side
jetsonTX2GPIONumber flow = gpio392; // flow from PLC
jetsonTX2GPIONumber level = gpio296; // level from PLC
jetsonTX2GPIONumber maintence = gpio481; //use water to clean from PLC
jetsonTX2GPIONumber start_stop = gpio254; //start_stop program 

jetsonTX2GPIONumber OFF = gpio430; //shut down everything
jetsonTX2GPIONumber emerg_stop = gpio297; //interrupt motors in place
jetsonTX2GPIONumber disable_Y = gpio467; //disable both Y motors
jetsonTX2GPIONumber disable_X_EE = gpio394; //disable both X and EE motors
jetsonTX2GPIONumber extra = gpio393; // not used

//disable : 1 = motors on, 0 = motors off


//----export and set direction of gpio -----
MotorInit(encoderL,dirYL,stepYL,encoderR,dirYR,stepYR,dirEE,stepEE,dirX,stepX,solenoidOut,solenoidIn,flow,level,maintence,start_stop,OFF,emerg_stop,disable_Y,disable_X_EE,extra);


//--------Sramp parameters---------
float d = 10; //predetermined
float linear_ratio = 0.75; //length of linear region compared to ramp of all motors
float step = 1600 / 9; //count per linear inch
float step_total = step * d; //total counts to travel
int up_down;
int ramp_step, ramp2_step, linear_step;
int pid_linear;

//-------YSramp parameters----------
gpioSetValue(dirYL,off); //CW
gpioSetValue(dirYR,on); //CCW

float height = 5;// move up 5 inches
float velo_y = 4.68; //whatever speed
int current_step;
int step_total_y = step  *height;
float linear_delay_y = 0.005625/velo_y;
bool pre_size = 1;


struct calc_Sramp_data x =  Sramp_function(height, velo_y, linear_ratio);//create calculations, store in struct
int linear_step_y = step_total_y - 2*x.step;

//--------ADC initiate----------
int i2cfile = init_i2c(); //pass file descriptor

//--------fifo file------------
const char * sizefifo = "/tmp/size"; //file locations
const char * speedfifo = "/tmp/speed";
const char * startfifo = "/tmp/start";
	
float CVspeed = 0; //initiate data
bool CVsize = 0;
bool CVstart = 0;

int fdsize = open(sizefifo, O_RDONLY); //open fifo file
cout<<"open fdsize"<<endl;
int fdspeed = open(speedfifo, O_RDONLY);
cout<<"open fdspeed"<<endl;
int fdstart = open(startfifo, O_RDONLY);
cout<<"open fdstart"<<endl;

//---------EE parameters---------
struct endeffector td;
td.STEP = stepEE;
td.solenoidIn = solenoidIn;
td.solenoidOut = solenoidOut;
td.spray = 0;

pthread_t tid;
pthread_attr_t attr;
pthread_attr_init(&attr);

//--------y encoders-------------
struct encoder EDL, EDR; //encoder data left/right
EDL.GPIO = encoderL;
EDR.GPIO = encoderR;

EDL.counter = 0;
EDR.counter = 0;

EDL.enable = 1;
EDR.enable = 1;

pthread_t YL, YR; // left/right y encoder

//----------initiate-------------
//move all to home position

/*
//start encoders
pthread_create(&YL, &attr, encoder, &EDL);
pthread_create(&YR, &attr, encoder, &EDR);

//move to large wreath position
//accelerate y motors
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

//linear y motors
current_step=0;

while(current_step<linear_step_y){
        if(EDL.counter == EDR.counter){
                gpioSetValue(stepYL,on);
                gpioSetValue(stepYR,on);
                usleep(1000000*linear_delay_y/2);
                gpioSetValue(stepYL,off);
                gpioSetValue(stepYR,off);
                usleep(1000000*linear_delay_y/2);
                current_step++;
        }
        else if(EDL.counter > EDR.counter){
                gpioSetValue(stepYR,on);
                usleep(1000000*linear_delay_y/2);
                gpioSetValue(stepYR,off);
                usleep(1000000*linear_delay_y/2);
        }
        else if(EDL.counter < EDR.counter){
                gpioSetValue(stepYL,on);
                usleep(1000000*linear_delay_y/2);
                gpioSetValue(stepYL,off);
                usleep(1000000*linear_delay_y/2);
        }
        else;

}

//deceleration
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


//end encoders
EDL.enable=0;
EDR.enable=0;
pthread_join(YR, NULL);
pthread_join(YL, NULL);
*/

//--------start loop (main code)-------------


//--------y motor----------------

/* //latch breaks when there is not new data, then it will move on to read values
CVdata=0;
while(CVdata==0){
read(fddata, &CVdata, sizeof(CVdata));
}
*/
for(int fifocount=0;fifocount<4;fifocount++){
//{
read(fdsize, &CVsize, sizeof(CVsize)); //CV velocity;
cout<<"Size = "<<CVsize<<endl;
read(fdspeed, &CVspeed, sizeof(CVspeed)); //CV velocity;
cout<<"Speed = "<<CVspeed<<endl;

cout<<"starting value "<<pre_size<<endl;
linear_step_y = 100; //change later (override value from above)
linear_delay_y = 0.005625/1; //change size at 1 inch per second (arbitrary value to not skip steps)

EDL.counter = 0; 
EDR.counter = 0;

EDL.enable = 1;
EDR.enable = 1;

if (pre_size != CVsize){
	if(CVsize == (1)){
	
	gpioSetValue(dirYL,on); //CW
	gpioSetValue(dirYR,off); //CCW

	pthread_create(&YL, &attr, encoder, &EDL);
	pthread_create(&YR, &attr, encoder, &EDR);
	
	current_step=0;
 	while(current_step<linear_step_y){
        	if(EDL.counter == EDR.counter){
                	gpioSetValue(stepYL,on);
	                gpioSetValue(stepYR,on);
	                usleep(1000000*linear_delay_y/2);
	                gpioSetValue(stepYL,off);
	                gpioSetValue(stepYR,off);
	                usleep(1000000*linear_delay_y/2);
	                current_step++;
	        	cout<<"large wreath "<<current_step<<endl;
		}
        	else if(EDL.counter > EDR.counter){
	                gpioSetValue(stepYR,on);
	                usleep(1000000*linear_delay_y/2);
	                gpioSetValue(stepYR,off);
	                usleep(1000000*linear_delay_y/2);
	 	}
	        else if(EDL.counter < EDR.counter){
	                gpioSetValue(stepYL,on);
	                usleep(1000000*linear_delay_y/2);
	                gpioSetValue(stepYL,off);
	                usleep(1000000*linear_delay_y/2);
        		}
		else;
	}//kill encoders until next move
	//y motors wont be moving until then
	EDL.enable=0; 
	EDR.enable=0;
	pthread_join(YR, NULL);
	pthread_join(YL, NULL);
	}

	else if(CVsize == (0)){
        
	gpioSetValue(dirYL,off); //CW
	gpioSetValue(dirYR,on); //CCW


	pthread_create(&YL, &attr, encoder, &EDL);
        pthread_create(&YR, &attr, encoder, &EDR);

	current_step=0;
        while(current_step<linear_step_y){
	        if(EDL.counter == EDR.counter){
	                gpioSetValue(stepYL,on);
	                gpioSetValue(stepYR,on);
	                usleep(1000000*linear_delay_y/2);
	                gpioSetValue(stepYL,off);
	                gpioSetValue(stepYR,off);
	                usleep(1000000*linear_delay_y/2);
	                current_step++;
			cout<<"small wreath "<<current_step<<endl;
	
		}
	        else if(EDL.counter > EDR.counter){
	                gpioSetValue(stepYR,on);
	                usleep(1000000*linear_delay_y/2);
        	        gpioSetValue(stepYR,off);
	                usleep(1000000*linear_delay_y/2);
		}
	        else if(EDL.counter < EDR.counter){
	                gpioSetValue(stepYL,on);
	                usleep(1000000*linear_delay_y/2);
        	        gpioSetValue(stepYL,off);
	                usleep(1000000*linear_delay_y/2);
			}

		else;
		
	}//kill encoders until next move
        //y motors wont be moving until then
        EDL.enable=0;
        EDR.enable=0;
        pthread_join(YR, NULL);
        pthread_join(YL, NULL);
        }
	else;
}
	
else;//belongs to if(pre_size != CVsize){
pre_size = CVsize;
cout<<"previous size = " <<pre_size<<endl;

}

//loop read CV start here
/* 
CVstart=0;
while(CVstart==0){
read(fdstart, &CVstart, sizeof(CVstart));
}
*/  

//--------X/EE motor and Solenoids------------
/*
gpioSetValue(disable_X_EE,on); // turn on motor

gpioSetValue(dirX,off); //on is CW, off is CW looking at the motor
gpioSetValue(dirEE,off); //on is CW, off is CW looking at the motor

usleep(100);
read(fdspeed, &CVspeed, sizeof(CVspeed)); //remove later
cout<<"Speed = "<<CVspeed<<endl;

read(fdsize, &CVsize, sizeof(CVsize));
cout<<"Size = "<<CVsize<<endl;
if(CVsize) td.spray = 2; //outter for larger wreath
else td.spray = 1; //inner for small wreath

td.time_total=d/CVspeed; //approximates total time to travel distance

pthread_create(&tid, &attr, EEMotor, &td); //create EE thread

up_down=1; // 1 = accelerate ; 0 = decelerate 
ramp_step = Sramp(up_down, d, CVspeed, linear_ratio, stepX); 
cout<<"ramp_step ="<<ramp_step<<endl;

pid_linear = 1; // 1 = pid ; 0 = linear
linear_step = linear_pid(pid_linear, CVspeed, ramp_step, step_total, i2cfile, stepX);

up_down=0;
ramp2_step = Sramp(up_down, d, CVspeed, linear_ratio, stepX); 

pthread_join(tid, NULL); //stop end effector


int actual_step = ramp_step + linear_step + ramp2_step;//reference for actual distance traveled
float actual_distance = actual_step/step;
cout<<"actual steps = "<<actual_step<<endl;
cout<<"actual distance = "<<actual_distance<<endl;
*/
//loop



//---------------------------------------------
signal(SIGINT, signalHandler);


	return 0;
}

	
