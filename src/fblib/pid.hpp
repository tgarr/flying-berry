
#ifndef _FBPID_HPP_
#define _FBPID_HPP_

#include "config.hpp"

class PID {
    float kp,ki,kd;
    float previous_error,integral;

public:
    PID(float,float,float);
    PID(float*);
    ~PID();

    float update(float,float,float);
    void reset();
};

#endif

