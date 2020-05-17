
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
    range = PWM_MAX - PWM_MIN;
    cur = 0;
    min_pulse = PWM_MIN + (range * fbconfig.min_throttle);
    max_pulse = PWM_MIN + (range * fbconfig.max_throttle);
    max_pulse_increase = range * fbconfig.max_throttle_increase;

    // PWM setup
    gpioSetPWMfrequency(pin,PWM_FREQUENCY);
    gpioSetPWMrange(pin,PWM_RANGE);
}

void Motor::on(){
    set_pulse(PWM_MIN);
    sleep(fbconfig.delay_on);
}

void Motor::off(){
    set_pulse(0);
}

// prevent ESCs from smoking
int Motor::protection(int p){
    if(p < PWM_MIN) p = PWM_MIN;
    if(p > PWM_MAX) p = PWM_MAX;
    if((p-cur) > max_pulse_increase) p = cur + max_pulse_increase;
    if(p < min_pulse) p = min_pulse;
    if(p > max_pulse) p = max_pulse;
    return p;
}

void Motor::throttle(float t){
    int thr = PWM_MIN + t*range;
    set_pulse(protection(thr));
}

void Motor::accelerate(float a){ 
    int thr = cur + (range * a);
    set_pulse(protection(thr));
}

void Motor::set_pulse(int p){
    gpioPWM(pin,p);
    cur = p;
}

float Motor::current(){
    return (cur-PWM_MIN) / (float)range;
}

void Motor::start_calibration(){
    calibrating = true;
}

void Motor::calibration_max(){
    if(calibrating) set_pulse(PWM_MAX);
}

void Motor::calibration_min(){
    if(calibrating) set_pulse(PWM_MIN);
}

void Motor::stop_calibration(){
    calibrating = false;
    set_pulse(0);
}

// arm all ESCs and sleep only once
void all_motors_on(Motor& fl,Motor& fr,Motor& bl,Motor& br){
    fl.set_pulse(PWM_MIN);
    fr.set_pulse(PWM_MIN);
    bl.set_pulse(PWM_MIN);
    br.set_pulse(PWM_MIN);
    sleep(fbconfig.delay_on);
}

void all_motors_on(Motor** m){
    all_motors_on(*m[0],*m[1],*m[2],*m[3]);
}

