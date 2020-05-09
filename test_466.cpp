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

int main(){
jetsonTX2GPIONumber dir = gpio466;

gpioExport(dir);
gpioSetDirection(dir, outputPin);

gpioSetValue(dir, low);
usleep(1000000);
gpioSetValue(dir, high);
usleep(1000000);
gpioUnexport(dir);

return 0;
}
