
#include "drone.hpp"

Drone::Drone(){
}

Drone::~Drone(){
}

bool Drone::setup(){
    //if(gpioInitialise() == PI_INIT_FAILED) return false;

    return true;
}

void Drone::run(){
}

void Drone::finalize(){
    // gpioTerminate();
}

