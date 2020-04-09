
#include "pid.hpp"

PID::PID(){
    reset();
}

PID::~PID(){
}

void PID::reset(){
    previous_error = 0;
    integral = 0;
}

float PID::update(float current,float setpoint,float dt){
    float error = setpoint - current;
    integral = integral + (error * dt);
    float derivative = (error - previous_error) / dt;

    // anti-windup
    if(integral > fbconfig.integral_limit) integral = fbconfig.integral_limit;
    if(integral < -fbconfig.integral_limit) integral = -fbconfig.integral_limit;

    // TODO organize class to apply appropriate K values (mode, pitch/roll/yaw)
    float pid = KP*error + KI*integral + KD*derivative;

    // saturation
    if(pid > fbconfig.pid_limit) pid = fbconfig.pid_limit;
    if(pid < -fbconfig.pid_limit) pid = -fbconfig.pid_limit;

    previous_error = error;
    return pid;
}


