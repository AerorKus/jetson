#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
using namespace std;
int main()
{

    const char * sizefifo = "/tmp/size";
    const char * speedfifo = "/tmp/speed";
    const char * startfifo = "/tmp/start";

    float speed;
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
    int fdspeed = open(speedfifo,O_WRONLY);
    int fdstart = open(startfifo,O_WRONLY);
	
    cout<<"Speed"<<endl;
    cin>>speed;
    write(fdsize, &size, sizeof(size));
    write(fdspeed, &speed,sizeof(speed));
    write(fdstart, &start,sizeof(start));

    for(int i=0; i<6000;i++){
    write(fdspeed, &speed,sizeof(speed));
    }
    close(fdsize);
    close(fdspeed);
    close(fdstart);
    /* remove the FIFO */
    usleep(10000000);
    unlink(sizefifo);
    unlink(speedfifo);
    unlink(startfifo);
return 0;

}
