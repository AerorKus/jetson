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
using namespace std;


/*
dont use fifo in thread: when looped fifo disappers, only reads the first time
	probably some memory relocating issue
dont use fifo in function: doesnt even detect first time
do pid and fifo reading in the main. since only x axis is fifo controlled
you can call read as many times
you cant call open as many times, must unlink first(might kill connection)
	prevent CV from sending
	call open once

*/

struct thread_data {
	int fdspeed;
	int count;
};


void* moveXMotor(void *threadarg){// {doesnt like it in a structure but ok by itself
        
	struct thread_data *my_data = (struct thread_data*) threadarg;
	
	float CVspeed=0;
	int fdspeed = my_data->fdspeed;

	for(int i=0;i<4;i++){
	read(fdspeed, &CVspeed, sizeof(CVspeed));
        if(CVspeed!=0){
        cout<<"Received speed:"<<CVspeed<<endl;}
        else
        cout<<"speed:none"<<endl;
	}
	
	pthread_exit(NULL);
	//return 0;
}

int XMotor(int fdspeed,float distance,float CVspeed=0){// {doesnt like it in a structure but ok by itself

        for(int i=0;i<4;i++){
        read(fdspeed, &CVspeed, sizeof(CVspeed));
        if(CVspeed!=0){
        cout<<"Received speed:"<<CVspeed<<endl;}
        else
        cout<<"speed:none"<<endl;
        }
return fdspeed;	
        }

int main(){
	
	struct thread_data td;	

	pthread_attr_t attr;
        pthread_attr_init(&attr);
	pthread_t tid;
	
	float distance=0;
    	float CVspeed=0;
	float CVsize=0;
        bool CVstart=0;
        const char *speedfifo="/tmp/speed";
	const char *sizefifo="/tmp/size";
        const char *startfifo="/tmp/start";

	//order matters of opening not reading. writing order must match reading order
	usleep(100);//delay length matters, fifo write is slower than read
	//for(int i=0;i<2;i++){
	//order: size>start>speed
	int fdsize = open(sizefifo, O_RDONLY);
	int fdspeed = open(speedfifo, O_RDONLY);
	td.fdspeed=fdspeed;
	//int fdspeed = open(speedfifo, O_RDONLY);
 	int fdstart = open(startfifo, O_RDONLY);
	
	for(int i=0;i<2;i++){//loop after open, or it will hang

        read(fdsize, &CVsize, sizeof(CVsize));
        if(CVsize!=0){
        cout<<"Received size:"<<CVsize<<endl;}
        else
        cout<<"size:none"<<endl;

	//pthread_create(&tid, &attr, moveXMotor, &td);
        //pthread_join(tid, NULL);
     	
	int fdspeed=XMotor(fdspeed,distance,CVspeed);
	

        read(fdstart, &CVstart, sizeof(CVstart));
        if(CVstart!=0){
        cout<<"Received start:"<<CVstart<<endl;}
        else
        cout<<"start:none"<<endl;


      	/*td.fdspeed=fdspeed;
	pthread_create(&tid, &attr, moveXMotor, &td);
        pthread_join(tid, NULL);
*/
        		
        /*read(fdspeed, &CVspeed, sizeof(CVspeed));
        if(CVspeed!=0){
        cout<<"Received speed:"<<CVspeed<<endl;}
        else
        cout<<"speed:none"<<endl;
*/

        read(fdstart, &CVstart, sizeof(CVstart));
        if(CVstart!=0){
        cout<<"Received start:"<<CVstart<<endl;}
        else
        cout<<"start:none"<<endl;

	read(fdsize, &CVsize, sizeof(CVsize));
        if(CVsize!=0){
        cout<<"Received size:"<<CVsize<<endl;}
        else
        cout<<"size:none"<<endl;
	
	cout<<"last"<<endl;
	
	close(fdsize);
	close(fdstart);
	close(td.fdspeed);
	/*close(fdspeed);
	unlink(sizefifo);//does unlinking prevent sending data from CV
  	unlink(speedfifo);
   	unlink(startfifo);
*/
	}
 	return 0;
}
