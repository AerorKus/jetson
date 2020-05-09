#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <inttypes.h>
#include <stdlib.h>
#include <errno.h>
using namespace std;

int main(){
	int file;//file descriptor
	int I2CAddress=0x48;//sudo i2cdetect -r -y 1 (in the shell)
        const char *filename = "/dev/i2c-1";//i2c bus 1 and location in the memory from root

	int16_t val;//16 bit value
	uint8_t config[3];//8 bit value
	uint8_t address[1];
	uint8_t read_buf[2];
	float data; //computed voltage from bits received

	float VPS=6.144/2048; //volts per step, used 2^11, 12th bit is negative bit for differential
	//opens i2c file 
	if ((file = open(filename, O_RDWR)) < 0) {
		/* ERROR HANDLING: you can check errno to see what went wrong */
		perror("Failed to open the i2c bus");
		exit(1);
	}
	//makes connection to bus
	if(ioctl(file,I2C_SLAVE,I2CAddress)<0){
		/* ERROR HANDLING; you can check errno to see what went wrong */
		perror("Failed to connect");
		exit(1);
	}
	//configuring i2c
	config[0]=0x01;//points to config register
	config[1]=0xC0;//MSB config register 
	config[2]=0x83;//LSB config register
	address[0]=0x00;//points to conversion register
	
	
	/* reading and writing automatically writes the slave address byte with the R/!W
	 everytime the slave address is needed, call another write/read,
	 otherwise just write consecutive bytes into the same array*/

	if(write(file, config,3)!=3){//if we dont pass all 3 indexes something went wrong
		perror("Write to config register");
		exit(1);
	}
	if(write(file, address,1)!=1){
                perror("Write to address register");
                exit(1);
        }
	usleep(100);//makes it more realiable 
		
	for(int i=0;i<50;i++){	
	read(file,read_buf,2);
	val = read_buf[0] <<4 | read_buf[1]>>4;
	usleep(100000);
	data=val*VPS;//steps x volts per steps = volts
        cout<<"value: "<<val<<"\n Data: "<<data<<"\n"<<endl;


	}
	/*bits [11:4] is in read_buf[0]
	bits [3:0] is in read_buf[1]
	shift first 8 up by 4 to make room for the last 4 bits
	11 10 9 8 7 6 5 4 
	 3  2 1 0 X X X X

	11 10 9 8 7 6 5 4 X X X X  -shifted left by 4
         	  3 2 1 0 X X X X	
	
	11 10 9 8 7 6 5 4 X X X X  
	                  3 2 1 0 -shifted right by 4

	Or will combine the two
	*/

	data=val*VPS;//steps x volts per steps = volts
	cout<<"value: "<<val<<"\n Data: "<<data<<"\n"<<endl;	
	
	close(file);
return 0; 
}

