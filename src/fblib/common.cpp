
#include "common.hpp"

bool fblib_initialise(){
    if(gpioInitialise() == PI_INIT_FAILED) return false;
    return true;
}

void fblib_finalize(){
    gpioTerminate();
}

