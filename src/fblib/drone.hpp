
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
    IMU* imu; // sensors
    Motor* motor[4]; // quadcopter
    PID* pid[2][3]; // PID control for stabilize/rate modes vs roll/pitch/yaw
    Control* control; // commands input
    FlightMode mode; // current flight mode
    int loop_interval; // duration of each update loop in microseconds
    float setpoints[3] = {0,0,0}; // current setpoints 

public:
    Drone();
    ~Drone();

    bool setup();
    void run();
    void finalize();
    void set_mode(FlightMode);
    void set_setpoints(float,float,float);
    float* get_setpoints();
};

#endif

