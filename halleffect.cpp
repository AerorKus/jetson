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


struct i2c_struct {
	float HE_X;
	float HE_Y;
	float HE_EE1; //home
	float HE_EE2; //if wrong way
	int enable;
	jetsontx2GPIO step_X;
	jetsontx2GPIO dir_X;
	jetsontx2GPIO step_EE;
	jetsontx2GPIO dir_EE;
};

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


void MotorInit(
jetsontx2GPIO dirEE,
jetsontx2GPIO stepEE,
jetsontx2GPIO dirX,
jetsontx2GPIO stepX
)
{

        cout << "exporting pins" << endl;

    // Make the button and led available in user space

    gpioExport(dirEE);
    gpioExport(stepEE);
    gpioExport(dirX);
    gpioExport(stepX);


	//set to input or output
	gpioSetDirection(dirEE, outputPin);
	gpioSetDirection(stepEE, outputPin);
	gpioSetDirection(dirX, outputPin);
	gpioSetDirection(stepX, outputPin);
}


void signalHandler(int signum) {
        cout << "Disabling motors\n";

    // cleanup and close up stuff here
    // terminate program
	//left side
	jetsonTX2GPIONumber dirEE = gpio427; // dir EE motor
	jetsonTX2GPIONumber stepEE = gpio398; // step EE motor
	jetsonTX2GPIONumber dirX = gpio298; // dir X motor
	jetsonTX2GPIONumber stepX = gpio389; // step X motor
	
	//Setvalue
        gpioSetValue(dirEE, low);
        gpioSetValue(stepEE, high);
        gpioSetValue(dirX, low);
        gpioSetValue(stepX, high);

	//Unexport      
        gpioUnexport(dirEE);     
		gpioUnexport(stepEE); 
		gpioUnexport(dirX);
        gpioUnexport(stepX);	
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
	cout<<"voltage A0 = "<<my_data->HE_X<<endl;	//A0
	
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
	cout<<"voltage A1 = "<<my_data->HE_EE1<<endl;	//A1	
	
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
	cout<<"voltage A2 = "<<my_data->HE_EE2<<endl; //A2	
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
    my_data->HE_Y = val*VPS;//steps x volts per steps = volts
	cout<<"voltage A3 = "<<my_data->HE_Y<<endl;	//A3
	
	usleep(500);//makes it more realiable

	}
close(file);
pthread_exit(NULL);
}

int main(int argc, char *argv[]){

//-----clean gpio-----------
signal(SIGINT, signalHandler);

//--------gpio pin---------
jetsonTX2GPIONumber dirEE = gpio427; // dir EE motor
jetsonTX2GPIONumber stepEE = gpio398; // step EE motor
jetsonTX2GPIONumber dirX = gpio298; // dir X motor
jetsonTX2GPIONumber stepX = gpio389; // step X motor


//----export and set direction of gpio -----
MotorInit(dirEE,stepEE,dirX,stepX);

//---------i2c initiate-----------
struct i2c_struct i2c_analog; //encoder data left/right
i2c_analog.enable = 1;


pthread_t i2c_thread;
pthread_attr_t attr;
pthread_attr_init(&attr);

//--------home parameters X/EE---------
pthread_t Xhome_td,EEhome_td;

i2c_analog.step_X = stepX;
i2c_analog.dir_X = dirX;

i2c_analog.step_EE = stepEE;
i2c_analog.dir_EE = dirEE;


//--------------code----------------
pthread_create(&i2c_thread, &attr, i2c_file, &i2c_analog);
//move all to home position

pthread_create(&Xhome_td, &attr, Xhome, &i2c_analog); //move X to home
pthread_create(&EEhome_td, &attr, EEhome, &i2c_analog);
usleep(1000000); //remove this


pthread_join(Xhome_td, NULL); //remove this
i2c_analog.enable = 0; 
pthread_join(i2c_thread, NULL);

signal(SIGINT, signalHandler);
return 0;
}

