# rpi_pigpio_wave_stepper
Waveform based acceleration controlled stepper driver for Raspberry Pi  
Can easily run on a Pi 3 with about 20% CPU usage.  
Capable of outputting 100k steps per second with 1 stepper or about 25k steps with 5 steppers  .
Requires pigpio and does not support other programs using GPIO pins at the same time. 
Must be run as root  

# Build and run:
    g++ -lstdc++ -Wall -pthread -o stepper *.cpp -lpigpio -lrt
    sudo ./stepper