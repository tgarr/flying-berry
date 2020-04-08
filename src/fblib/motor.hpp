
#ifndef _FBMOTOR_HPP_
#define _FBMOTOR_HPP_

#include <pigpio.h>
#include "config.hpp"

enum class MotorPosition { front_left = FL, front_right = FR, back_left = BL, back_right = BR };

class Motor {
    int pin,range,cur;

    void setup(int);
    void set_pulse(int);

public:
    Motor(int);
    Motor(MotorPosition);
    ~Motor();

    void on();
    void off();
    void throttle(float);
    void accelerate(float);
    float current();

    friend void all_motors_on(Motor&,Motor&,Motor&,Motor&);
};

#endif

