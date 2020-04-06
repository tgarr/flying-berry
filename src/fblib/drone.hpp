
#ifndef _FBDRONE_HPP_
#define _FBDRONE_HPP_

#include <pigpio.h>
#include "common.hpp"
#include "config.hpp"
#include "control.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "pid.hpp"

class Drone {

public:
    Drone();
    ~Drone();

    bool setup();
    void run();
    void finalize();
};

#endif

