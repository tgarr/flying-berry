
#include "drone.hpp"

Drone::Drone(Controller* c){
    loop_interval = 1.0f / fbconfig.looprate * 1000 * 1000;
    mode = fbconfig.default_mode;
    controller = c;
    c->set_drone(this);
}

Drone::~Drone(){
}

bool Drone::setup(){
    if(!fblib_initialise()) return false;
    
    // initialise IMU
    imu = new IMU;
    if(!imu->ok) return false;

    // initialise motors
    motor[static_cast<int>(MotorPosition::front_left)] = new Motor(MotorPosition::front_left);
    motor[static_cast<int>(MotorPosition::front_right)] = new Motor(MotorPosition::front_right);
    motor[static_cast<int>(MotorPosition::back_left)] = new Motor(MotorPosition::back_left);
    motor[static_cast<int>(MotorPosition::back_right)] = new Motor(MotorPosition::back_right);

    // avoid sleeping too much
    fbconfig.delay_on = fbconfig.delay_on - fbconfig.calibration_time;
    if(fbconfig.delay_on < 0) fbconfig.delay_on = 0;

    // arm the motors
    std::cout << "Turning motors on ... " << std::flush; // XXX
    all_motors_on(motor);
    std::cout << "done" << std::endl; // XXX

    // calibrate IMU
    std::cout << "Calibrating IMU ... " << std::flush; // XXX
    imu->calibrate();
    std::cout << "done" << std::endl; // XXX

    // create PID system
    for(int i=0;i<2;i++) // stabilize and rate modes
        for(int j=0;j<3;j++) // rate, pitch, and yaw
            pid[i][j] = new PID(fbconfig.pid[i][j]);

    return true;
}

void Drone::main(){
    float dt,pidv[3] = {0,0,0}; 
    struct timeval *st = new struct timeval;
    struct timeval *et = new struct timeval;
    struct timeval *temp;
    int looptime = 0;

    gettimeofday(et,NULL); 
    while(!end){
        // delta time
        temp = st; st = et; et = temp;
        dt = looptime / 1000000.0f;

        // update attitude
        imu->update(dt);
        Angle* angles = imu->attitude();
        //std::cout << "Roll: " << angles[ROLL] << " | Pitch: " << angles[PITCH] << " | Yaw: " << angles[YAW] << std::endl; // XXX

        // update PID
        for(int i=0;i<3;i++) pidv[i] = pid[static_cast<int>(mode)][i]->update(angles[i],setpoints[i],dt) * fbconfig.pid_multiplier;

        // actuate on motors
        update_motors(pidv);

        // check commands
        controller->update(dt);
        //std::cout << "Roll: " << roll() << " Pitch: " << pitch() << " Throttle: " << throttle() << std::endl; // XXX

        // sleep until next update loop 
        gettimeofday(et,NULL);
        int elapsed = ((et->tv_sec - st->tv_sec)*1000000) + (et->tv_usec - st->tv_usec);
        int remaining = loop_interval - elapsed;
        if(remaining > 0) usleep(remaining);

        // compute looptime
        gettimeofday(et,NULL);
        looptime = ((et->tv_sec - st->tv_sec)*1000000) + (et->tv_usec - st->tv_usec);
        //std::cout << looptime << " " << elapsed << std::endl; // XXX
    }
    
    delete st;
    delete et;
}

// update motors throttle
void Drone::update_motors(float *pidv){
    if(!flying) return;

    //std::cout << "PID roll: " << pidv[ROLL] << " | PID pitch: " << pidv[PITCH] << " | Throttle: " << setpoints[THROTTLE] << std::endl; // XXX

    float fl = setpoints[THROTTLE] + pidv[ROLL] + pidv[PITCH] - pidv[YAW]; // front left
    float fr = setpoints[THROTTLE] - pidv[ROLL] + pidv[PITCH] + pidv[YAW]; // front right
    float bl = setpoints[THROTTLE] + pidv[ROLL] - pidv[PITCH] + pidv[YAW]; // back left
    float br = setpoints[THROTTLE] - pidv[ROLL] - pidv[PITCH] - pidv[YAW]; // back right

    motor[static_cast<int>(MotorPosition::back_right)]->throttle(br);
    motor[static_cast<int>(MotorPosition::front_left)]->throttle(fl);
    motor[static_cast<int>(MotorPosition::front_right)]->throttle(fr);
    motor[static_cast<int>(MotorPosition::back_left)]->throttle(bl);
}

void Drone::finalize(){
    delete imu;
    delete controller;

    for(int i=0;i<4;i++) delete motor[i];
    
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            delete pid[i][j];
    
    fblib_finalize();
}

void Drone::set_mode(FlightMode m){
    mode = m;
    for(int i=0;i<3;i++){
        pid[static_cast<int>(mode)][i]->reset();
        setpoints[i] = 0;
    }
}

void Drone::panic(){
    set_mode(FlightMode::stabilize);
}

void Drone::descend(){
    // TODO
}

void Drone::start(){
    if(flying) return;
    for(int i=0;i<3;i++){
        pid[static_cast<int>(mode)][i]->reset();
        setpoints[i] = 0;
    }
    imu->reset();
    flying = true;
    throttle(fbconfig.min_base_throttle);
    std::cout << "Starting!" << std::endl;
}

void Drone::roll(float r){
    if(!flying) return;
    if(mode == FlightMode::stabilize){
        if(r < -fbconfig.stab_max_roll) r = -fbconfig.stab_max_roll;
        if(r > fbconfig.stab_max_roll) r = fbconfig.stab_max_roll;
        setpoints[ROLL] = r;
    }
    else setpoints[ROLL] += r;
}

void Drone::pitch(float p){
    if(!flying) return;
    if(mode == FlightMode::stabilize){
        if(p < -fbconfig.stab_max_pitch) p = -fbconfig.stab_max_pitch;
        if(p > fbconfig.stab_max_pitch) p = fbconfig.stab_max_pitch;
        setpoints[PITCH] = p;
    }
    else setpoints[PITCH] += p;
}

void Drone::yaw(float y){
    if(!flying) return;
    setpoints[YAW] += y; // always in rate mode
}

void Drone::throttle(float t){
    if(!flying) return;
    if(t < fbconfig.min_base_throttle) t = fbconfig.min_base_throttle;
    if(t > fbconfig.max_base_throttle) t = fbconfig.max_base_throttle;
    setpoints[THROTTLE] = t;
}

void Drone::set_setpoints(float r,float p,float y,float t){
    roll(r);
    pitch(p);
    yaw(y);
    throttle(t);
}

void Drone::stop(){
    flying = false;
    end = true;
}

