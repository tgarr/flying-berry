
#ifndef _FBCONTROLLER_HPP_
#define _FBCONTROLLER_HPP_

#include "config.hpp"
#include "drone.hpp"

// cross-reference
class Drone;

class Controller {
protected:
    Drone* drone;

public:
    virtual ~Controller(){}
    virtual void update(float) = 0;
    void set_drone(Drone* d){ drone = d; }
};

#endif

