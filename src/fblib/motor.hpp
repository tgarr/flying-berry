
#ifndef _FBMOTOR_HPP_
#define _FBMOTOR_HPP_

#include <pigpio.h>
#include "config.hpp"

#define PWM_RANGE 2500
#define PWM_FREQUENCY 400
#define PWM_MIN 1000
#define PWM_MAX 2000

class Motor {
    int pin,range,cur,min_pulse,max_pulse,max_pulse_increase;
    bool calibrating = false;

    void setup(int);
    void set_pulse(int);
    int protection(int);

public:
    Motor(int);
    Motor(MotorPosition);
    ~Motor();

    void on();
    void off();
    void throttle(float);
    void accelerate(float);
    float current();
    void start_calibration();
    void stop_calibration();
    void calibration_min();
    void calibration_max();

    friend void all_motors_on(Motor&,Motor&,Motor&,Motor&);
    friend void all_motors_on(Motor**);
};

#endif

