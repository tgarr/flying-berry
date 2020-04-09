
#include "drone.hpp"

Drone::Drone(){
    loop_interval = 1.0f / fbconfig.looprate * 1000 * 1000;
    mode = fbconfig.default_mode;
}

Drone::~Drone(){
}

bool Drone::setup(){
    if(gpioInitialise() == PI_INIT_FAILED) return false;

    // initialise motors
    motor[static_cast<int>(MotorPosition::front_left)] = new Motor(MotorPosition::front_left);
    motor[static_cast<int>(MotorPosition::front_right)] = new Motor(MotorPosition::front_right);
    motor[static_cast<int>(MotorPosition::back_left)] = new Motor(MotorPosition::back_left);
    motor[static_cast<int>(MotorPosition::back_right)] = new Motor(MotorPosition::back_right);
    all_motors_on(motor);

    // initialise IMU
    imu = new IMU;
    if(!imu->ok) return false;
    imu->calibrate();

    // create PID system
    for(int i=0;i<2;i++) // stabilize and rate modes
        for(int j=0;j<3;j++) // rate, pitch, and yaw
            pid[i][j] = new PID(fbconfig.pid[i][j]);

    // control
    control = new Control(); // TODO parameters

    return true;
}

void Drone::run(){
    float dt;
    struct timeval st,et; // TODO use pointers: switch st/et at the start of loop

    int looptime = 0;
    int i = 0;    
    while(1){ // TODO end condition

        // delta time
        gettimeofday(&st,NULL); // TODO use pointer: st = et; et = st;
        dt = looptime / 1000000.0f;

        // update attitude
        imu->update(dt);

        // XXX print attidude
        i++;
        if((i*loop_interval) % (500*1000) == 0){
            Angle* angles = imu->attitude();
            std::cout << angles[ROLL] << " " << angles[PITCH] << " " << angles[YAW] << std::endl;
        }

        // TODO update PID
        // TODO update motors
        // TODO execute command (change setpoints)

        // sleep until next update loop 
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec-st.tv_sec)*1000000)+(et.tv_usec-st.tv_usec);
        int remaining = loop_interval-elapsed;
        if(remaining > 0) usleep(remaining);

        // compute looptime
        gettimeofday(&et,NULL);
        looptime = ((et.tv_sec-st.tv_sec)*1000000)+(et.tv_usec-st.tv_usec);
    }
}

void Drone::finalize(){
    delete imu;
    delete control;

    for(int i=0;i<4;i++) delete motor[i];
    
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            delete pid[i][j];
    
    gpioTerminate();
}

void Drone::set_mode(FlightMode m){
    mode = m;
    for(int i=0;i<3;i++){
        pid[static_cast<int>(mode)][i]->reset();
        setpoints[i] = 0;
    }
}

void Drone::set_setpoints(float r,float p,float y){
    setpoints[ROLL] = r;
    setpoints[PITCH] = p;
    setpoints[YAW] = y;
}

float* Drone::get_setpoints(){
    return setpoints;
}

