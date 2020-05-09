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

/*CV will give us a size and velocity at the same time before it reaches gantry
 *therefore, y first then EE and X at the same time 
 */
//used to pass arguments into the background thread
struct thread_data {
        jetsontx2GPIO STEP;
        jetsontx2GPIO DIRECTION;
	float velocity;
	float distance;//inches  
	float linear_ratio;//how long linear compared to Sramp
};

/*thread the EE motor, x motor and y motor can be from the Sramp function
 * EE will include solenoid driver operation 
 * Sramp - accelerate and decelerate and linear ( no pid )
 * XPID only part of main, because fifo needs to be called in main
 * Y counts encoder pulses to determine position
 * 	should return steps in array for comparing with encoder
 * 	thread to move y motors, main to read encoder
 * 	compare encoder value to steps taken in y thread
 * 	can loop it from run thread> count encoder> join thread> compare> run again until equal
 * 	need to fill array up to meet position
 * 		do normal sramp and fill up the rest with linear
 * 		how switch from small to large
 * 			only ramps? make it fixed transistion		
 * 
 * interrupts will do what? kernel space talk to userspace
 * 	what will it do in kernel? 
 * 	where will it resume when it finishes kernel thing
 * 	i could sample between steps for limits in linear
 * 		will never interrupt during ramp up unless super slow
 *	
 *	
 * how much clearance for deceleration ramp? 	
 *	units: cm 
 *	so we can measure for the comparator Vref
 * 
*/
//background thread function that runs horizontal motors
void *moveMotor(void* threadarg) {
	struct thread_data* my_data;
        my_data = (struct thread_data*) threadarg;

        //initialize function variables with data from the structure
        
        jetsontx2GPIO STEP = my_data->STEP;
	jetsontx2GPIO DIRECTION = my_data->DIRECTION;
	float v_CV = my_data->velocity;//inches per second
	float d = my_data->distance;//inches
	float linear_ratio=my_data->linear_ratio;

        float v=0;
        float delay,ta,a,time_total,curve_ratio,seg_time;
        float time;
        float sarray[2000];

        time_total=(d)/v_CV;
        curve_ratio=(1-linear_ratio)/4.0;
        cout<<curve_ratio<<"\n"<<endl;

        ta=2.0*curve_ratio*time_total;
        a=2*v_CV/ta;
        cout<<a<<"\n"<<endl;
        cout<<"ta="<<ta<<"\n"<<endl;

        seg_time=ta/2;
        cout<<"seg_time="<<seg_time<<endl;

        time=seg_time/9;

        int s=0;
        int counta=0, countb=0;

        while(time<ta)
        {
                if(time<=seg_time){
                        v=a*a/(2*v_CV)*(time*time);
                        //cout<<v<<endl;
                        //cout<<va="<<v<<endl;
                        counta=counta+1;
                }
                else if(time>seg_time&&time<=ta){
                        v=v_CV-a*a/(2*v_CV)*(ta-time)*(ta-time);
                        //cout<<v<<endl;
                        //cout<<"vb="<<v<<endl;
                        countb=countb+1;
                }

		 	delay=0.00403/v;
                        sarray[s]=delay;
                        time=time+delay;
                        s++;
                	cout<<delay<<endl;

        }

        float linear_delay=0.00403/v_CV;
        time=0;
        float linear_time = linear_ratio*time_total;
        cout<<s<<endl;
        for(int i=0;i<s;i++){
                gpioSetValue(STEP,on);
                usleep(1000000*sarray[i]/2);
                gpioSetValue(STEP,off);
                usleep(1000000*sarray[i]/2);
                cout<<"up "<<1000000*sarray[i]/2<<endl;
        }

        while(time<linear_time){
                gpioSetValue(STEP,on);
                usleep(1000000*linear_delay/2);
                gpioSetValue(STEP,off);
                usleep(1000000*linear_delay/2);
                time=time+linear_delay;
                cout<<"linear "<<1000000*linear_delay/2<<endl;
        }

        for(int i=s-1;i>0;i--){
                gpioSetValue(STEP,on);
                usleep(1000000*sarray[i]/2);
                gpioSetValue(STEP,off);
                usleep(1000000*sarray[i]/2);
                cout<<"down "<<1000000*sarray[i]/2<<endl;
        }

	pthread_exit(NULL);
}

//use to ramp all motors
int Sramp(bool up_down, float v_CV, float d, float linear_ratio,jetsontx2GPIO STEP, jetsontx2GPIO DIRECTION) {
     
        
        float v=0;
        float delay,ta,a,time_total,curve_ratio,seg_time;
        float time;
        float sarray[2000];

        time_total = d/v_CV;
        curve_ratio=(1-linear_ratio)/4.0;
        cout<<curve_ratio<<"\n"<<endl;

        ta=2.0*curve_ratio*time_total;
        a=2*v_CV/ta;
        cout<<a<<"\n"<<endl;
        cout<<"ta="<<ta<<"\n"<<endl;

        seg_time=ta/2;
        cout<<"seg_time="<<seg_time<<endl;

        time=seg_time/9;

        int s=0;
        int counta=0, countb=0;

        while(time<ta)
        {
                if(time<=seg_time){
                        v=a*a/(2*v_CV)*(time*time);
                        //cout<<v<<endl;
                        //cout<<va="<<v<<endl;
                        counta=counta+1;
                }


 		else if(time>seg_time&&time<=ta){
                        v=v_CV-a*a/(2*v_CV)*(ta-time)*(ta-time);
                        //cout<<v<<endl;
                        //cout<<"vb="<<v<<endl;
                        countb=countb+1;
                }

                        delay=0.00403/v;
                        sarray[s]=delay;
                        time=time+delay;
                        s++;
                        cout<<delay<<endl;

        }

       	if(up_down){// ramp up is 1, ramp down is 0

        for(int i=0;i<s;i++){
                gpioSetValue(STEP,on);
                usleep(1000000*sarray[i]/2);
                gpioSetValue(STEP,off);
                usleep(1000000*sarray[i]/2);
                cout<<"up "<<1000000*sarray[i]/2<<endl;
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
        
}
return s-1;
}



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
        jetsonTX2GPIONumber ena = gpio396;     // Output - xaxis motors
        jetsonTX2GPIONumber step = gpio392; // Output
        jetsonTX2GPIONumber dir = gpio255; // Output
        jetsonTX2GPIONumber ena120 = gpio398;     // Output - EE motors
        jetsonTX2GPIONumber step120 = gpio298; // Output
        jetsonTX2GPIONumber dir120 = gpio389; // Output

        gpioSetValue(ena, low);
        gpioSetValue(dir, low);
        gpioSetValue(step, low);
        gpioSetValue(ena120, low);
        gpioSetValue(dir120, low);
        gpioSetValue(step120, low);

        gpioUnexport(ena);     // unexport the ENA
        gpioUnexport(dir);      // unexport the DIR
        gpioUnexport(step);      // unexport the STEP
        gpioUnexport(ena120);     // unexport the ENA
        gpioUnexport(dir120);      // unexport the DIR
        gpioUnexport(step120);      // unexport the STEP
        
	exit(signum);
}


int main(int argc, char *argv[]){

        // register signal SIGINT and signal handler
        signal(SIGINT, signalHandler);//put unlink fifos in here


        //for FIFO
        float CVspeed=0;
        float CVsize=0;
        bool CVstart=0;
        const char *speedfifo="/tmp/speed";
        const char *sizefifo="/tmp/size";
        const char *startfifo="/tmp/start";


        //allows access to the stucture in main
        struct thread_data td;


        //thread ID
        pthread_t tid;
        // Create attributes
        pthread_attr_t attr;
        pthread_attr_init(&attr);

	
	//initialize gpio pins
        jetsonTX2GPIONumber ENA = gpio396 ;     // Output - xaxis motor
        
	jetsonTX2GPIONumber STEP = gpio392 ; // Output
        jetsonTX2GPIONumber DIRX = gpio255; // Output
        jetsonTX2GPIONumber ENA120 = gpio398;     // Output - EE motors
        jetsonTX2GPIONumber STEP120 = gpio298; // Output
        jetsonTX2GPIONumber DIR120 = gpio389; // Output

	//iniate fifo and motor
        MotorInit(ENA, STEP, DIRX);//put mkfifo in here
        MotorInit(ENA120, STEP120, DIR120);

        td.STEP = STEP;
        td.DIRECTION=DIRX;
	td.distance=24;
	td.linear_ratio=0.75;
	

        int fdsize = open(sizefifo, O_RDONLY);
        int fdspeed = open(speedfifo, O_RDONLY);
	int fdstart = open(startfifo, O_RDONLY);
	

        read(fdsize, &CVsize, sizeof(CVsize));
        if(CVsize!=0){
        cout<<"Received:"<<CVsize<<endl;}
        else
        cout<<"none"<<endl;
        

        read(fdspeed, &CVspeed, sizeof(CVspeed));
        if(CVspeed!=0){
        cout<<"Received:"<<CVspeed<<endl;}
        else
        cout<<"none"<<endl;
        

        read(fdstart, &CVstart, sizeof(CVstart));
        if(CVstart!=0){
        cout<<"Received:"<<CVstart<<endl;}
        else
        cout<<"none"<<endl;

	td.velocity=CVspeed;

                        gpioSetValue(DIRX, high);
                        gpioSetValue(DIR120, high);

                        //creates thread to run x-axis motor in one direction
                        pthread_create(&tid, &attr, moveMotor, &td);

 
                        //waits until background thread is terminated
                        pthread_join(tid, NULL);

                        gpioSetValue(DIRX, low);
                        gpioSetValue(DIR120, low);

                        //creates thread to run x-axis motor in the other direction
                        //pthread_create(&tid, &attr, moveMotor, &td);

                        //run EE motor again
                        //moveEEMotor(STEP120, ENA120, 40000, 2000);

                        //pthread_join(tid, NULL);

	close(fdsize);
	close(fdspeed);
	close(fdstart);

    return 0;
}
