
#ifndef _FBDRONE_HPP_
#define _FBDRONE_HPP_

#include <pigpio.h>
#include <sys/time.h>
#include <unistd.h>
#include "common.hpp"
#include "config.hpp"
#include "controller.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "pid.hpp"

// cross-reference
class Controller;

class Drone {
    IMU* imu; // sensors
    Motor* motor[4]; // quadcopter
    PID* pid[2][3]; // PID control for stabilize/rate modes vs roll/pitch/yaw
    Controller* controller; // commands input
    FlightMode mode; // current flight mode
    int loop_interval; // duration of each update loop in microseconds
    float setpoints[4] = {0,0,0,0}; // current setpoints 
    bool end = false;
    bool flying = false;

    void update_motors(float*);
public:
    Drone(Controller*);
    ~Drone();

    bool setup();
    void main();
    void finalize();
    void start();
    void stop();

    void panic();
    void descend();
    void set_mode(FlightMode);
    bool is_flying(){ return flying; }
    
    float* get_setpoints(){ return setpoints; }
    void set_setpoints(float,float,float,float);

    float roll(){ return setpoints[ROLL]; }
    void roll(float);
    
    float pitch(){ return setpoints[PITCH]; }
    void pitch(float);
    
    float yaw(){ return setpoints[YAW]; }
    void yaw(float);
    
    float throttle(){ return setpoints[THROTTLE]; }
    void throttle(float);
};

#endif

