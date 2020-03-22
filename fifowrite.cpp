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

    float speed=20.20;
    float size=10.10;
    bool start=1;
    unlink(sizefifo);
    unlink(speedfifo);
    unlink(startfifo);
    /* create the FIFO (named pipe) */
    mkfifo(sizefifo, 0666);
    mkfifo(speedfifo, 0666);
    mkfifo(startfifo, 0666);
    /* write "Hi" to the FIFO */
    int fdsize = open(sizefifo, O_WRONLY);
    write(fdsize, &size, sizeof(size));
    close(fdsize);

    int fdspeed = open(speedfifo,O_WRONLY);
    write(fdspeed, &speed,sizeof(speed));
    close(fdspeed);

    int fdstart = open(startfifo,O_WRONLY);
    write(fdstart, &start,sizeof(start));
    close(fdstart);

    /* remove the FIFO */
    usleep(10000);
    unlink(sizefifo);
    unlink(speedfifo);
    unlink(startfifo);
return 0;

}
