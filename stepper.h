#ifndef STEPPER_H
#define STEPPER_H

#define MAX_PULSES 12000

#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <pigpio.h>
#include <vector>
#include <chrono>
#include <thread>
#include <math.h>

using namespace std;
using namespace std::this_thread;
using namespace std::chrono_literals;
using namespace std::chrono;  
using std::chrono::system_clock;

typedef long long us_t;

us_t time_us();

struct step_pulse
{
    int up;
    int down;
};

class Stepper
{
public:
    Stepper(int dir, int step, double accel, double max_spd);

    double speed = 0.0;
    double acc = 0.0;
    double max_speed = 0.0;

    int position = 0; //current position of stepper
    int target = 0; //target position to move stepper
    double target_speed = NAN; //hold constant speed, overrides target

    void wait_motion_end();
    void wait_speed_reached();

    us_t _get_next_state_change_us(); //these are called by waveform transmitter, not for general use
    void _update_speed(us_t current_us);
    step_pulse _step_now(us_t current_us);

private:
    int pin_dir = 0;
    int pin_step = 0;

    us_t last_step_us = 0;
    us_t last_speed_update_us = 0;

    int last_dir = 2;
    int state = 0;
};

class StepperWaveformTransmitter
{
public:
    StepperWaveformTransmitter(us_t plan_length_us, int enable_p);
    void add_stepper(Stepper* stepper);
    void start();
    void stop();
    void next_wf();
private:
    void thread_loop();

    us_t planning_len = 0;

    int enable_pin = 0;

    int next_id = 0;

    us_t last_tx_us = 0;
    us_t last_tx_len = 0;

    vector<Stepper *> steppers;
    vector<int> waves;

    thread* gen_thread = nullptr;

    bool stop_flag = false;
};

#endif