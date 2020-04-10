
#ifndef _FBCONTROL_HPP_
#define _FBCONTROL_HPP_

#include "config.hpp"
#include "drone.hpp"

// cross-reference
class Drone;

class Control {
    Drone* drone;

public:
    Control(Drone*);
    ~Control();

    void update(float);
};

#endif

