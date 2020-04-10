
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

#define PID_SCALE 0.01

// cross-reference
class Control;

class Drone {
    IMU* imu; // sensors
    Motor* motor[4]; // quadcopter
    PID* pid[2][3]; // PID control for stabilize/rate modes vs roll/pitch/yaw
    Control* control; // commands input
    FlightMode mode; // current flight mode
    int loop_interval; // duration of each update loop in microseconds
    float setpoints[4] = {0,0,0,0}; // current setpoints 
    bool end = false;
    bool flying = false;

    void update_motors(float*);
public:
    Drone();
    ~Drone();

    bool setup();
    void run();
    void finalize();
    void set_mode(FlightMode);
    void set_setpoints(float,float,float,float);
    float* get_setpoints();
    void stop();
};

#endif

