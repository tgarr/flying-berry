
#ifndef _FBPID_HPP_
#define _FBPID_HPP_

#include "config.hpp"

class PID {
    float previous_error,integral;

public:
    PID();
    ~PID();

    float update(float,float,float);
    void reset();
};

#endif

