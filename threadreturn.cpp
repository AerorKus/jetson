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
using namespace std;


/*testing how to pass thread data to main
 *use structure 
 *	assign values in the thread to the structure 
 *	then call the structure in the main
*/
struct thread_data {
	int in;
	int out;
};


void *thread(void* threadarg){
	struct thread_data *my_data = (struct thread_data*) threadarg;

	int input = my_data->in;
	cout<<input<<endl;
	my_data->out = 4;

pthread_exit(NULL);
}

int main(){
	
struct thread_data td;
td.in=10;

//thread ID
pthread_t tid;
// Create attributes
pthread_attr_t attr;
pthread_attr_init(&attr);

pthread_create(&tid, &attr,thread, &td);
pthread_join(tid, NULL);
cout<<td.out<<endl;

	return 0;
}
