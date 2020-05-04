
#include "common.hpp"

bool fblib_initialise(){
    // pigpio sample rate
    //gpioCfgClock(5,1,0); // cfgMicros, cfgPeripheral, cfgSource (ignored)

    if(gpioInitialise() == PI_INIT_FAILED) return false;
    return true;
}

void fblib_finalize(){
    gpioTerminate();
}

