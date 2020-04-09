
#include "pid.hpp"

PID::PID(float p,float i,float d){
    kp = p;
    ki = i;
    kd = d;
    reset();
}

PID::PID(float* k){
    kp = k[static_cast<int>(PIDTerm::KP)];
    ki = k[static_cast<int>(PIDTerm::KI)];
    kd = k[static_cast<int>(PIDTerm::KD)];
    reset();
}

PID::~PID(){
}

void PID::reset(){
    previous_error = 0;
    integral = 0;
}

float PID::update(float current,float setpoint,float dt){
    // PID terms
    float error = setpoint - current;
    integral = integral + (error * dt);
    float derivative = (error - previous_error) / dt;

    // anti-windup
    if(integral > fbconfig.integral_limit) integral = fbconfig.integral_limit;
    if(integral < -fbconfig.integral_limit) integral = -fbconfig.integral_limit;
    
    // final value
    float pid = kp*error + ki*integral + kd*derivative;

    // saturation
    if(pid > fbconfig.pid_limit) pid = fbconfig.pid_limit;
    if(pid < -fbconfig.pid_limit) pid = -fbconfig.pid_limit;

    previous_error = error;
    return pid;
}


