
#ifndef _FBDRONE_HPP_
#define _FBDRONE_HPP_

#include <pigpio.h>
#include <sys/time.h>
#include <unistd.h>
#include "common.hpp"
#include "config.hpp"
#include "control.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "pid.hpp"

class Drone {
    IMU* imu;

    int loop_interval;

public:
    Drone();
    ~Drone();

    bool setup();
    void run();
    void finalize();
};

#endif

