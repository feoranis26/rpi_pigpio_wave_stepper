#include "stepper.h"

us_t time_us()
{
    struct timeval tv;
    gettimeofday(&tv, 0);

    return (us_t)(((double)tv.tv_sec * 1000000) + (double)tv.tv_usec);
}

Stepper::Stepper(int dir, int step, double accel, double max_spd)
{
    pin_dir = dir;
    pin_step = step;

    acc = accel;
    max_speed = max_spd;

    last_step_us = time_us();
    last_speed_update_us = time_us();

    gpioSetMode(pin_step, PI_OUTPUT);
    gpioSetMode(pin_dir, PI_OUTPUT);

    gpioWrite(pin_step, 0);
    gpioWrite(pin_dir, 0);
}

void Stepper::wait_motion_end(){
    while(position != target)
        sleep_for(milliseconds(10));
}

void Stepper::wait_speed_reached(){
    while(abs(speed - target_speed) > 5)
        sleep_for(milliseconds(10));
}

us_t Stepper::_get_next_state_change_us()
{
    if (abs(speed) <= 1.0)
        return 0;

    us_t us_per_step = (us_t)(1000000.0 / abs(speed));
    // printf("Next step: %lld\n", us_per_step + last_step_us);
    // printf("Last step: %lld\n", last_step_us);
    return us_per_step + last_step_us;
}

void Stepper::_update_speed(us_t current_us)
{
    double time_delta = (current_us - last_speed_update_us) / 1000000.0;
    last_speed_update_us = current_us;

    if (!isnan(target_speed))
    {
        if (speed > target_speed)
            speed -= acc * time_delta;
        else
            speed += acc * time_delta;

        if(last_dir == 2)
        {
            gpioWrite(pin_dir, speed < 0);
            last_dir = speed < 0;
        }
        return;
    }

    if (position == target)
    {
        speed = 0;
        return;
    }

    double dist_to_stop = (speed * speed / (acc * 2)) * (speed > 0 ? 1 : -1);

    // printf("Time delta: %f\n", time_delta);
    // printf("Speed: %f\n", speed);

    if (position + dist_to_stop > target)
        speed -= acc * time_delta;
    else
        speed += acc * time_delta;

    speed = min(max(-max_speed, speed), max_speed);
}

step_pulse Stepper::_step_now(us_t current_us)
{
    last_step_us = current_us;

    step_pulse pulse;
    pulse.up = 0;
    pulse.down = 0;

    if (state == 0)
        pulse.up |= 1 << pin_step;
    else if (state == 1)
        pulse.down |= 1 << pin_step;

    state = !state;

    if (speed < 0 && last_dir == 1)
        pulse.up |= 1 << pin_dir;
    else if (speed > 0 && last_dir == 0)
        pulse.down |= 1 << pin_dir;

    last_dir = speed > 0;
    // printf("Position: %d\n", position);
    // printf("Speed: %f\n", speed);

    if (speed != 0)
        position += speed > 0 ? 1 : -1;

    // printf("Speed: %f\n", speed);
    // printf("US: %lld\n", current_us);

    if (position == target && isnan(target_speed))
    {
        speed = 0;
        return pulse;
    }

    // printf("Pulse up: %d, down: %d\n", pulse.up, pulse.down);
    // printf("Pulse up: %d\n", 1 << pin_step);
    // printf("State: %d\n", state);
    return pulse;
}

StepperWaveformTransmitter::StepperWaveformTransmitter(us_t plan_length_us, int enable_p)
{
    planning_len = plan_length_us;

    enable_pin = enable_p;

    last_tx_us = 0;
    last_tx_len = 0;

    gpioSetMode(enable_pin, PI_OUTPUT);
    gpioWrite(enable_pin, 0);

    waves.assign(2, PI_NO_TX_WAVE);
}

void StepperWaveformTransmitter::add_stepper(Stepper *stepper)
{
    steppers.push_back(stepper);
}

void StepperWaveformTransmitter::start(){
    printf("Starting thread!\n");
    printf("This: %p", this);
    gen_thread = new thread(&StepperWaveformTransmitter::thread_loop, this);
}

void StepperWaveformTransmitter::stop(){
    printf("Signalling thread to stop!\n");
    stop_flag = true;
    gen_thread->join();

    gpioWrite(enable_pin, 0);
}

void StepperWaveformTransmitter::next_wf()
{
    int old_id = gpioWaveTxAt();

    us_t start_us = last_tx_us + last_tx_len;

    if (abs(time_us() - start_us) - planning_len > 1000) //not enough time to plan current cycle!
    {
        printf("Missed start time! Should be %lld us earlier!\n", abs(time_us() - start_us) - planning_len - 1000);
        start_us = time_us();
    }

    us_t current_us = 0;

    gpioPulse_t *pulses = (gpioPulse_t *)malloc(sizeof(gpioPulse_t) * MAX_PULSES);
    int num_pulses = 0;

    for (Stepper *stepper : steppers)
        stepper->_update_speed(start_us);

    us_t last_speed_update_us = start_us;

    while (current_us < planning_len && num_pulses < MAX_PULSES)
    {
        Stepper *stepper_to_step = nullptr;
        us_t next_step_us = INT64_MAX;

        for (Stepper *stepper : steppers)
        {
            us_t step_us = stepper->_get_next_state_change_us();

            if (step_us != 0 and step_us < next_step_us)
            {
                next_step_us = step_us;
                stepper_to_step = stepper;
            }
        }

        if (stepper_to_step == nullptr) //no pulses left for current waveform
            break;

        us_t delta_us = next_step_us - (start_us + current_us);

        if (delta_us < 0)
        {
            if (delta_us < -1000)
                printf("Too late! Should be %lld us earlier\n", -delta_us);

            delta_us = 0;
        }

        if (delta_us > 1000) //To ensure that stepper speeds are updated at least once every 1ms
            delta_us = 1000;

        current_us += delta_us;

        if (current_us > planning_len)
            break;

        if ((current_us + start_us) - last_speed_update_us > 1000)
        {
            for (Stepper *stepper : steppers)
                stepper->_update_speed(current_us + start_us);

            last_speed_update_us = start_us + current_us;
        }

        gpioPulse_t delay_pulse;

        delay_pulse.gpioOn = 0;
        delay_pulse.gpioOff = 0;
        delay_pulse.usDelay = (int)(delta_us / 2);
        pulses[num_pulses++] = delay_pulse;

        int pulse_up = 0;
        int pulse_down = 0;

        while (true)
        {
            stepper_to_step = nullptr;

            for (Stepper *stepper : steppers)
            {
                us_t step_us = stepper->_get_next_state_change_us();

                if (step_us <= current_us + start_us && step_us != 0)
                {
                    step_pulse stp_pulse = stepper->_step_now(start_us + current_us);

                    pulse_up |= stp_pulse.up;
                    pulse_down |= stp_pulse.down;

                    stepper_to_step = stepper;
                }
            }

            if (stepper_to_step == nullptr)
                break;
        }

        gpioPulse_t full_pulse;
        full_pulse.gpioOn = pulse_up;
        full_pulse.gpioOff = pulse_down;
        full_pulse.usDelay = 0;

        pulses[num_pulses++] = full_pulse;
    }

    if (num_pulses == 0) //No pulses in current waveform, wait until start of next one
    {
        us_t wait_us = start_us - time_us();
        sleep_for(microseconds(wait_us - 5000));

        while (gpioWaveTxBusy())
            gpioDelay(10);

        if (old_id != PI_NO_TX_WAVE)
            gpioWaveDelete(old_id);

        last_tx_us = time_us();
        gpioWrite(enable_pin, 0);

        return;
    }
    else
        gpioWrite(enable_pin, 1);

    if (old_id != PI_NO_TX_WAVE)
        gpioWaveDelete(old_id);

    gpioWaveAddGeneric(num_pulses, pulses);
    delete pulses;

    int new_wave = gpioWaveCreatePad(50, 50, 0);

    us_t wait_us = start_us - time_us();

    sleep_for(microseconds(wait_us - 5000)); //wait until 5ms before next wave starts

    while (gpioWaveTxBusy()) //spin until end of current wave
        gpioDelay(10);

    if (time_us() - last_tx_us > planning_len + 1000) //check if cycle was quick enough
        printf("Total cycle took %lld us, %lld more than normal!\n", time_us() - last_tx_us, (time_us() - last_tx_us) - planning_len);

    last_tx_us = time_us();
    last_tx_len = current_us;

    gpioWaveTxSend(new_wave, PI_WAVE_MODE_ONE_SHOT);
}

void StepperWaveformTransmitter::thread_loop(){
    printf("Thread started.\n");
    printf("This: %p", this);
    while(!stop_flag)
        next_wf();
    printf("Thread stopped.\n");
}