
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
    max_pulse = fbconfig.esc_min_value + (range * fbconfig.max_throttle);
    max_pulse_increase = range * fbconfig.max_throttle_increase;
    set_pulse(0);
}

void Motor::on(){
    set_pulse(fbconfig.esc_min_value);
    sleep(fbconfig.delay_on);
}

void Motor::off(){
    set_pulse(0);
}

// prevent ESCs from smoking
int Motor::protection(int p){
    if(p < fbconfig.esc_min_value) p = fbconfig.esc_min_value;
    if(p > fbconfig.esc_max_value) p = fbconfig.esc_max_value;
    if((p-cur) > max_pulse_increase) p = cur + max_pulse_increase;
    if(p > max_pulse) p = max_pulse;
    return p;
}

void Motor::throttle(float t){
    int thr = fbconfig.esc_min_value + t*range;
    set_pulse(protection(thr));
}

void Motor::accelerate(float a){ 
    int thr = cur + (range * a);
    set_pulse(protection(thr));
}

void Motor::set_pulse(int p){
    gpioServo(pin,p);
    cur = p;
}

float Motor::current(){
    return (cur-fbconfig.esc_min_value) / (float)range;
}

void Motor::start_calibration(){
    calibrating = true;
}

void Motor::calibration_max(){
    if(calibrating) set_pulse(fbconfig.esc_max_value);
}

void Motor::calibration_min(){
    if(calibrating) set_pulse(fbconfig.esc_min_value);
}

void Motor::stop_calibration(){
    calibrating = false;
    set_pulse(0);
}

// arm all ESCs and sleep only once
void all_motors_on(Motor& fl,Motor& fr,Motor& bl,Motor& br){
    fl.set_pulse(fbconfig.esc_min_value);
    fr.set_pulse(fbconfig.esc_min_value);
    bl.set_pulse(fbconfig.esc_min_value);
    br.set_pulse(fbconfig.esc_min_value);
    sleep(fbconfig.delay_on);
}

void all_motors_on(Motor** m){
    all_motors_on(*m[0],*m[1],*m[2],*m[3]);
}

