#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

int main()
{

    const char * sizefifo = "/tmp/size";
    const char * speedfifo = "/tmp/speed";
    const char * startfifo = "/tmp/start";
	const char * datafifo = "/tmp/data";

    float speed=3;
    bool size=0; // 0 small, 1 large
    bool start=1;
	bool data=1;

    unlink(sizefifo);
    unlink(speedfifo);
    unlink(startfifo);
	unlink(datafifo);
    /* create the FIFO (named pipe) */
    mkfifo(sizefifo, 0666);
    mkfifo(speedfifo, 0666);
    mkfifo(startfifo, 0666);
    mkfifo(datafifo, 0666);

    int fdsize = open(sizefifo, O_WRONLY);
 	int fdspeed = open(speedfifo,O_WRONLY);
    int fdstart = open(startfifo,O_WRONLY);
	int fddata = open(datafifo,O_WRONLY);

    write(fdsize, &size, sizeof(size));
    size=1;
    write(fdsize, &size, sizeof(size));

	write(fdspeed, &speed,sizeof(speed));

	write(fdstart, &start,sizeof(start));
	write(fdstart, &start,sizeof(start));
	write(fddata, &data,sizeof(data));	
	write(fddata, &data,sizeof(data));	

    close(fdstart);
    close(fdsize);
    close(fdspeed);
    /* remove the FIFO */
    usleep(100000);
    unlink(sizefifo);
    unlink(speedfifo);
    unlink(startfifo);
	unlink(datafifo);
return 0;

}
