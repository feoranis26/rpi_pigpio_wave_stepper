#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <pigpio.h>
#include <vector>
#include <chrono>
#include <thread>

#include "stepper.h"

using namespace std;
StepperWaveformTransmitter *tx;


void stop(){
    tx->stop();
    exit(1);
}


int main()
{
    gpioInitialise();
    gpioWaveClear();
    gpioWaveTxStop();

    Stepper *stepper1 = new Stepper(12, 6, 1000, 10000);
    Stepper *stepper2 = new Stepper(16, 13, 1000, 10000);
    stepper1->target = 100000;
    stepper2->target_speed = -10000;

    tx = new StepperWaveformTransmitter(100000, 5);

    tx->add_stepper(stepper1);
    tx->add_stepper(stepper2);

    //while(true)
    //    tx->next_wf();

    tx->start();

    stepper1->wait_motion_end();
    printf("motion end!\n");
    stepper1->target = 0;
    stepper2->target_speed = 1000;
    stepper1->wait_motion_end();
    stepper2->wait_speed_reached();
    stepper2->target_speed = 0;
    stepper2->wait_speed_reached();

    printf("speed reached!\n");

    tx->stop();
}