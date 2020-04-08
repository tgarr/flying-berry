
#include "motor.hpp"

Motor::Motor(MotorPosition pos){
    int p = static_cast<int>(pos);
    setup(fbconfig.esc_pin[p]);
}

Motor::Motor(int p){
    setup(p);
}

Motor::~Motor(){
    off();
}

void Motor::setup(int p){
    pin = p;
    range = fbconfig.esc_max_value - fbconfig.esc_min_value;
    cur = 0;
    set_pulse(0);
    max_pulse = fbconfig.esc_min_value + (range * (fbconfig.max_throttle/100.0f));
    max_pulse_increase = range * (fbconfig.max_throttle_increase/100.0f);
}

void Motor::on(){
    set_pulse(fbconfig.esc_min_value);
    sleep(5);
}

void Motor::off(){
    set_pulse(0);
}

void Motor::throttle(float t){
    int thr = fbconfig.esc_min_value + t*range;
    if(thr < fbconfig.esc_min_value) thr = fbconfig.esc_min_value;
    if(thr > fbconfig.esc_max_value) thr = fbconfig.esc_max_value;
    set_pulse(thr);
}

void Motor::accelerate(float a){ 
    int thr = cur + (range * a);
    if(thr < fbconfig.esc_min_value) thr = fbconfig.esc_min_value;
    if(thr > fbconfig.esc_max_value) thr = fbconfig.esc_max_value;
    set_pulse(thr);
}

void Motor::set_pulse(int p){
    // prevent ESCs from smoking
    if((p-cur) > max_pulse_increase) p = cur + max_pulse_increase;
    if(p > max_pulse) p = max_pulse;

    gpioServo(pin,p);
    cur = p;
}

float Motor::current(){
    return (cur-fbconfig.esc_min_value) / (float)range;
}

// arm all ESCs and sleep only once
void all_motors_on(Motor& fl,Motor& fr,Motor& bl,Motor& br){
    fl.set_pulse(fbconfig.esc_min_value);
    fr.set_pulse(fbconfig.esc_min_value);
    bl.set_pulse(fbconfig.esc_min_value);
    br.set_pulse(fbconfig.esc_min_value);
    sleep(5);
}

