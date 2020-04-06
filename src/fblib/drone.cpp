
#include "drone.hpp"

Drone::Drone(){
    loop_interval = 1.0f / fbconfig.looprate * 1000 * 1000;
}

Drone::~Drone(){
}

bool Drone::setup(){
    if(gpioInitialise() == PI_INIT_FAILED) return false;
    
    // initialise IMU
    imu = new IMU;
    if(!imu->ok) return false;
    imu->calibrate();


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
    gpioTerminate();
}

