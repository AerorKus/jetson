#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
using namespace std;

int main(){
	int file;
	int I2CAddress=0x40;
	const char *filename = "/dev/i2c-2";
	char buf[10]={0};
	float data;
	char channel;
	if ((file = open(filename, O_RDWR)) < 0) {
		/* ERROR HANDLING: you can check errno to see what went wrong */
		perror("Failed to open the i2c bus");
		exit(1);
	}
	if(ioctl(file,I2C_SLAVE,I2CAddress)<0){
		cout<<"Failed to acquire bus access and/or talk to slave.\n"<<endl;
		/* ERROR HANDLING; you can check errno to see what went wrong */
		     exit(1);
	}
	int n;
	n=read(file,buf,2);
	cout<<n<<endl;
	close(file);
return 0; 
}

