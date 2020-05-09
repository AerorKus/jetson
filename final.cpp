/*

//-----------how to run the program------------------
need sudo permission to operate gpio and i2c pins (./ofifotest | sudo ./ofinal ) 
must be ran with fifotest.cpp file: program takes inputs of fifo to run

//------------things to do when built-------------
PID needs to be tuned
hall effect voltages need to be tuned to see what voltage value determines that the home position i reached

//----------------todo---------------------
measure the height of wreath : put in initiation
measure difference between wreaths : how far to switch between
change distance traveled for x axis : 3s around 4.68 in/s
add return

//----------troubleshooting----------------
reasons to stall: fifo no value, adc not connected
gpio pin 466 doesnt work on this jetson

*/
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

struct i2c_struct {
	float HE_X;
	float HE_EE1; //home
	float HE_EE2; //if wrong way
	float FV;
	int enable;
	jetsontx2GPIO step_X;
	jetsontx2GPIO dir_X;
	jetsontx2GPIO step_EE;
	jetsontx2GPIO dir_EE;
};


void *EEhome(void* threadarg){ 
	struct i2c_struct* voltage_data = (struct i2c_struct*) threadarg;
			
	jetsontx2GPIO STEP = voltage_data->step_EE;
	jetsontx2GPIO DIR = voltage_data->dir_EE;
	float linear_velocity = 1; //1 inch per second
	float linear_delay=0.005625/linear_velocity;

	gpioSetValue(DIR,on);
	while(voltage_data->HE_EE1<4){ //tune this value: assuming 4V when hall effects detects metal 
		
		if(voltage_data->HE_EE2>=4){ //if other hall effects gets triggered move the other way (were going in the wrong direction)
			gpioSetValue(DIR,off); 	//move CW
		}	
	
		gpioSetValue(STEP,on);
        usleep(1000000*linear_delay/2);
    	gpioSetValue(STEP,off);
        usleep(1000000*linear_delay/2); 
	}	
	
pthread_exit(NULL);	
}

void *Xhome(void* threadarg){ 
	struct i2c_struct* voltage_data = (struct i2c_struct*) threadarg;
	
	jetsontx2GPIO STEP = voltage_data->step_X;
	jetsontx2GPIO DIR = voltage_data->dir_X;
	float linear_velocity = 1; //1 inch per second
	float linear_delay=0.005625/linear_velocity;
	gpioSetValue(DIR,on); 
	while(voltage_data->HE_X<4){ //tune this value: assuming 4V when hall effects detects metal 
	cout<<"voltage of hall effect read in Xhome "<<voltage_data->HE_X<<endl;
		gpioSetValue(STEP,on);
        usleep(1000000*linear_delay/2);
    	gpioSetValue(STEP,off);
        usleep(1000000*linear_delay/2);
	}	
pthread_exit(NULL);	
}


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
	//cout<<"count = "<<my_data->counter<<endl;
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


void *i2c_file(void* threadarg){ //A0(datasheet) is A3(on silkscreen) ; A1 = A0 ; A2 = A1 ; A3 = A2

	struct i2c_struct* my_data = (struct i2c_struct*) threadarg;
	int file;
	int I2CAddress=0x48;
    const char *filename = "/dev/i2c-1";
	uint8_t config[3];
	uint8_t address[1];
	uint8_t read_buf[2];

	float VPS=6.144/2048;	
	int16_t val;

	
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
	usleep(500);

	while(my_data->enable){

	//read A0
	config[0]=0x01;//points to config register
	config[1]=0xD0;//MSB config register 1101 0000 FSR=6.144, AIN1 to GND
	config[2]=0x83;//LSB config register
	address[0]=0x00;//points to conversion register

	write(file, config,3);
	write(file, address,1);
	read(file,read_buf,2);

	val = read_buf[0] <<4 | read_buf[1]>>4;
    my_data->HE_X = val*VPS;//steps x volts per steps = volts
	//cout<<"voltage A0 = "<<my_data->HE_X<<endl;	//A0
	
	usleep(500);//makes it more realiable

	//read A1
	config[0]=0x01;//points to config register
	config[1]=0xE0;//MSB config register 1110 0000 FSR=6.144, AIN2 to GND
	config[2]=0x83;//LSB config register
	address[0]=0x00;//points to conversion register
	
	write(file, config,3);
	write(file, address,1);
	read(file,read_buf,2);

	val = read_buf[0] <<4 | read_buf[1]>>4;
    my_data->HE_EE1 = val*VPS;//steps x volts per steps = volts
	//cout<<"voltage A1 = "<<my_data->HE_EE1<<endl;	//A1	
	
	usleep(500);//makes it more realiable

	//read A2
	config[0]=0x01;//points to config register
	config[1]=0xF0;//MSB config register 1111 0000 FSR=6.144, AIN3 to GND
	config[2]=0x83;//LSB config register
	address[0]=0x00;//points to conversion register
	
	write(file, config,3);
	write(file, address,1);
	read(file,read_buf,2);

	val = read_buf[0] <<4 | read_buf[1]>>4;
    my_data->HE_EE2 = val*VPS;//steps x volts per steps = volts
	//cout<<"voltage A2 = "<<my_data->HE_EE2<<endl; //A2	
	usleep(500);//makes it more realiable
	
	//read A3
	config[0]=0x01;//points to config register
	config[1]=0xC0;//MSB config register 1100 0000 FSR=6.144, AIN0 to GND
	config[2]=0x83;//LSB config register 1000 0011
	address[0]=0x00;//points to conversion register
	
	write(file, config,3);
	write(file, address,1);
	read(file,read_buf,2);

	val = read_buf[0] <<4 | read_buf[1]>>4;
    my_data->FV = val*VPS;//steps x volts per steps = volts
	//cout<<"voltage A3 = "<<my_data->FV<<endl;	//A3
	
	usleep(500);//makes it more realiable

	}
close(file);
pthread_exit(NULL);
}

//used for calculations return in main function
void Y_initiate (int up_down, int CVsize, float linear_ratio, struct encoder EDL, struct encoder EDR, jetsontx2GPIO stepYL, jetsontx2GPIO stepYR, jetsontx2GPIO dirYL, jetsontx2GPIO dirYR){

float v,delay,ta,a,time_total,curve_ratio,seg_time,time;
int current_step,linear_step_y,step_total_y;
float sarray[2000];

int step = 0;
float v_CV = 3;
float large = 5;// move up 5 inches
float small = large + 3; //diameter of large : 28 ; small : 22 ; (28-22)/2
float linear_delay_y = 0.005625/v_CV;



if(CVsize){
	step_total_y = 1600*large/9;
	time_total=large/v_CV;
}
else{
	step_total_y = 1600*small/9;
	time_total=small/v_CV;
}

curve_ratio=(1-linear_ratio)/4.0;
ta=2.0*curve_ratio*time_total;
a=2*v_CV/ta;
seg_time=ta/2;
time=seg_time/9;

if(up_down){ //up = 1, down = 0
	gpioSetValue(dirYL,on); //CW
	gpioSetValue(dirYR,off); //CCW
}
else{
    gpioSetValue(dirYL,off); //CCW
    gpioSetValue(dirYR,on); //CW
}

while(time<ta){
        if(time<=seg_time){
                v=a*a/(2*v_CV)*(time*time);
                }
        else if(time>seg_time&&time<=ta){
                v=v_CV-a*a/(2*v_CV)*(ta-time)*(ta-time);
                }
                delay=0.005625/v;
                sarray[step]=delay;
                time=time+delay;
                step++;
        }
linear_step_y = step_total_y - 2*step;
//accelerate y motors
current_step=0;
while(current_step<step){ //correct place wrong time
	cout<<"EDL = "<<EDL.counter<<endl;
	cout<<"EDR = "<<EDR.counter<<endl;
        if(EDL.counter == EDR.counter){
                gpioSetValue(stepYL,on);
                gpioSetValue(stepYR,on);
                usleep(1000000*sarray[current_step]/2);
                gpioSetValue(stepYL,off);
                gpioSetValue(stepYR,off);
                usleep(1000000*sarray[current_step]/2);
                current_step++;
        }
        else if(EDL.counter > EDR.counter){
                gpioSetValue(stepYR,on);
                usleep(1000000*sarray[current_step]/2);
                gpioSetValue(stepYR,off);
                usleep(1000000*sarray[current_step]/2);
        }
        else if(EDL.counter < EDR.counter){
                gpioSetValue(stepYL,on);
                usleep(1000000*sarray[current_step]/2);
                gpioSetValue(stepYL,off);
                usleep(1000000*sarray[current_step]/2);
        }
        else;

}

//linear y motors
current_step=0;

while(current_step<linear_step_y){
		cout<<"EDL = "<<EDL.counter<<endl;
		cout<<"EDR = "<<EDR.counter<<endl;
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
while(current_step<step){ //correct place wrong time
		cout<<"EDL = "<<EDL.counter<<endl;
		cout<<"EDR = "<<EDR.counter<<endl;
        if(EDL.counter == EDR.counter){
                gpioSetValue(stepYL,on);
                gpioSetValue(stepYR,on);
                usleep(1000000*sarray[step-1-current_step]/2);
                gpioSetValue(stepYL,off);
                gpioSetValue(stepYR,off);
                usleep(1000000*sarray[step-1-current_step]/2);
                current_step++;
        }
        else if(EDL.counter > EDR.counter){
                gpioSetValue(stepYR,on);
                usleep(1000000*sarray[step-1-current_step]/2);
                gpioSetValue(stepYR,off);
                usleep(1000000*sarray[step-1-current_step]/2);
        }
        else if(EDL.counter < EDR.counter){
                gpioSetValue(stepYL,on);
                usleep(1000000*sarray[step-1-current_step]/2);
                gpioSetValue(stepYL,off);
                usleep(1000000*sarray[step-1-current_step]/2);
        }
        else;

}

}

//used for to move motors
int Sramp(int up_down,float d, float v_CV,float linear_ratio,jetsontx2GPIO STEP ){
	
	float v=0;
	float delay,ta,a,time_total,curve_ratio,seg_time;
	float time;
	float sarray[2000];

	time_total=d/v_CV;
 	curve_ratio=(1-linear_ratio)/4.0;
	
	ta=2.0*curve_ratio*time_total;
	a=2*v_CV/ta;

	seg_time=ta/2;

	time=seg_time/9;
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

int linear_pid(int pid_linear, double CVspeed, int ramp_step, int step_total, jetsontx2GPIO STEP,struct i2c_struct){
	//-------i2c---------
	float vPV = 2 / 1.2; //velocity per volt MUST CHANGE LATER
	
	//--------closed loop-------
    struct i2c_struct x;
	int current_step=0;
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
			//put voltage value from i2c here
	        pv = (6 - x.FV)*vPV;//process variable, volts x velocity per volts
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
jetsonTX2GPIONumber start_stop = gpio392; // Start from PLC
jetsonTX2GPIONumber OFF_ON = gpio296; // OFF from PLC
jetsonTX2GPIONumber maintenance = gpio481; //use water to clean from PLC
jetsonTX2GPIONumber emerg_stop = gpio254; //emergency stop 

jetsonTX2GPIONumber disable_Y = gpio430; //disables Y
jetsonTX2GPIONumber disable_X_EE = gpio297; //diables EE and X
jetsonTX2GPIONumber Extra1 = gpio467; //not used
jetsonTX2GPIONumber Extra2 = gpio394; //not used
jetsonTX2GPIONumber Extra3 = gpio393; // not used

//disable : 1 = motors on, 0 = motors off


//----export and set direction of gpio -----
MotorInit(encoderL,dirYL,stepYL,encoderR,dirYR,stepYR,dirEE,stepEE,dirX,stepX,solenoidOut,solenoidIn,start_stop,OFF_ON,maintenance,emerg_stop,disable_Y,disable_X_EE,Extra1,Extra2,Extra3);

//---------i2c initiate and home parameters X/EE---------
struct i2c_struct i2c_analog; //hall effect and FV converter data for home 
i2c_analog.enable = 1;

pthread_t i2c_thread;
pthread_t Xhome_td, EEhome_td;

i2c_analog.step_X = stepX;
i2c_analog.dir_X = dirX;

i2c_analog.step_EE = stepEE;
i2c_analog.dir_EE = dirEE;



//--------Sramp parameters---------
float d = 10; //predetermined
float linear_ratio = 0.75; //length of linear region compared to ramp of all motors
float step = 1600 / 9; //count per linear inch
float step_total = step * d; //total counts to travel
int up_down;
int ramp_step, ramp2_step, linear_step;
int pid_linear;
float return_50;

//-------YSramp parameters----------
int current_step;
int linear_step_y;
float linear_delay_y;
bool pre_size=1; //starts at large wreath
int ascend_descend;
int size_wreath;

//--------fifo file------------
const char * sizefifo = "/tmp/size"; //file locations
const char * speedfifo = "/tmp/speed";
const char * startfifo = "/tmp/start";
const char * datafifo = "/tmp/data";
	
float CVspeed = 0; //initiate data
bool CVsize = 0;
bool CVstart = 0;
bool CVdata = 0;

int fdsize = open(sizefifo, O_RDONLY); //open fifo file
cout<<"open fdsize"<<endl;
int fdspeed = open(speedfifo, O_RDONLY);
cout<<"open fdspeed"<<endl;
int fdstart = open(startfifo, O_RDONLY);
cout<<"open fdstart"<<endl;
int fddata = open(datafifo, O_RDONLY);

//---------PLC Buttons----------
unsigned int START = 0; //store read value here for start/stop
unsigned int OFF= 0; //OFF
unsigned int MAINTENANCE= 0; //cleaning 

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

//---------start code---------------

pthread_create(&i2c_thread, &attr, i2c_file, &i2c_analog);
//move all to home position

/*
pthread_create(&Xhome_td, &attr, Xhome, &i2c_analog); //move X to home
pthread_create(&EEhome_td, &attr, EEhome, &i2c_analog); //move EE to home
*/

//start encoders
pthread_create(&YL, &attr, encoder, &EDL);
pthread_create(&YR, &attr, encoder, &EDR);

//move to large wreath position
ascend_descend = 1; //go up
size_wreath = 1; //initiate to large wreath

Y_initiate(ascend_descend,size_wreath,linear_ratio,EDL,EDR,stepYL,stepYR,dirYL,dirYR);

//end encoders
EDL.enable=0;
EDR.enable=0;
pthread_join(YR, NULL);
pthread_join(YL, NULL);

/*
pthread_join(Xhome_td, NULL);
pthread_join(EEhome_td, NULL);
*/

/*
//--------start loop (main code)-------------


//--------y motor----------------

//loop goes here

//if stop, poll until want to start or turn off, prevents first movement when not wanted
gpioGetValue(start_stop, &START); //pin for start from PLC
while(START==1){ //start is 0 and stop is 1, while stopped stall program until start again
	gpioGetValue(start_stop, &START);
	gpioGetValue(OFF_ON, &OFF);
		if(START==0) break; //break loop on start
		if(OFF==0) break;
		usleep(100); 
	}
cout<<"START" <<START<<endl;
cout<<"OFF" <<OFF<<endl;

while(OFF==1){
//latch breaks when there is not new data, then it will move on to read values
CVdata=0;
while(CVdata==0){
read(fddata, &CVdata, sizeof(CVdata));
}

read(fdsize, &CVsize, sizeof(CVsize)); //CV velocity;
cout<<"Size = "<<CVsize<<endl;
read(fdspeed, &CVspeed, sizeof(CVspeed)); //CV velocity;
cout<<"Speed = "<<CVspeed<<endl;

cout<<"starting value "<<pre_size<<endl;
linear_step_y = 534; 
linear_delay_y = 0.005625/1; //change size at 1 inch per second (arbitrary value to not skip steps)

EDL.counter = 0; 
EDR.counter = 0;

EDL.enable = 1;
EDR.enable = 1;

if (pre_size != CVsize){
	if(CVsize == (1)){
	//move to large wreath from small (going down)
	gpioSetValue(dirYL,on); //CCW
	gpioSetValue(dirYR,off); //CW

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
	
else;//belongs to if(pre_size != CVsize)
pre_size = CVsize;

//loop read CV start here

CVstart=0;
while(CVstart==0){
read(fdstart, &CVstart, sizeof(CVstart));
}


//--------X/EE motor and Solenoids------------


gpioSetValue(dirX,off); //on is CW, off is CW looking at the motor
gpioSetValue(dirEE,off); //on is CW, off is CW looking at the motor


if(CVsize) td.spray = 2; //outter for larger wreath
else td.spray = 1; //inner for small wreath

td.time_total=d/CVspeed; //approximates total time to travel distance for EE thread
pthread_create(&tid, &attr, EEMotor, &td); //create EE thread

up_down=1; // 1 = accelerate ; 0 = decelerate 
ramp_step = Sramp(up_down, d, CVspeed, linear_ratio, stepX); 
cout<<"ramp_step ="<<ramp_step<<endl;

pid_linear = 0; // 1 = pid ; 0 = linear
linear_step = linear_pid(pid_linear, CVspeed, ramp_step, step_total, stepX,i2c_analog);

up_down=0;
ramp2_step = Sramp(up_down, d, CVspeed, linear_ratio, stepX); 
cout<<"ramp_step2 ="<<ramp2_step<<endl;

pthread_join(tid, NULL); //stop end effector


int actual_step = ramp_step + linear_step + ramp2_step;//reference for actual distance traveled
float actual_distance = actual_step/step;

//reverse

gpioSetValue(dirX,on); //on is CCW
gpioSetValue(dirEE,on); //on is CCW

return_50 = CVspeed*1.5; //move back 50% faster

td.time_total = actual_distance/return_50; //approximates total time to travel distance for EE thread
pthread_create(&tid, &attr, EEMotor, &td); //create EE thread

up_down=1; // 1 = accelerate ; 0 = decelerate 
int ramp_step_r = Sramp(up_down, actual_distance, return_50, linear_ratio, stepX); 
cout<<"ramp_step ="<<ramp_step_r<<endl;


pid_linear = 0; // 1 = pid ; 0 = linear
int linear_step_r = linear_pid(pid_linear, return_50, ramp_step, actual_step, stepX,i2c_analog);

up_down=0;
int ramp2_step_r = Sramp(up_down, actual_distance, return_50, linear_ratio, stepX); 
cout<<"ramp_step2 ="<<ramp2_step_r<<endl;

pthread_join(tid, NULL); //stop end effector

int actual_step_r = ramp_step + linear_step + ramp2_step;//reference for actual distance traveled
float actual_distance_r = actual_step/step;


gpioGetValue(start_stop, &START); //pin for start from PLC, gives first value for loop
while(START==1){ //start is 0 and stop is 1, while stopped stall program until start again
	gpioGetValue(start_stop, &START);
	gpioGetValue(OFF_ON, &OFF);
		if(START==0) break; //break loop on start
		if(OFF==0) break; //break loop on OFF
		usleep(100); 
	}
cout<<"START " <<START<<endl;
cout<<"OFF " <<OFF<<endl;

} //loop until off
 cout<<"OFF"<<endl;


//return back to home

pthread_create(&Xhome_td, &attr, Xhome, &i2c_analog); //move X to home
pthread_create(&EEhome_td, &attr, EEhome, &i2c_analog); //move EE to home


//start encoders
EDL.counter = 0;
EDR.counter = 0;

EDL.enable = 1;
EDR.enable = 1;

pthread_create(&YL, &attr, encoder, &EDL);
pthread_create(&YR, &attr, encoder, &EDR);

//move back down
ascend_descend = 0; //go up

Y_initiate(ascend_descend,CVsize,linear_ratio,EDL,EDR,stepYL,stepYR,dirYL,dirYR);

//end encoders
EDL.enable=0;
EDR.enable=0;
pthread_join(YR, NULL);
pthread_join(YL, NULL);

pthread_join(Xhome_td, NULL);
pthread_join(EEhome_td, NULL);

i2c_analog.enable = 0; 
pthread_join(i2c_thread, NULL);
*/
//---------------------------------------------
signal(SIGINT, signalHandler);
close(fdsize);
close(fdspeed);
close(fdstart);

	return 0;
}

	
