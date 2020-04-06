
#ifndef _FBCONFIG_HPP_
#define _FBCONFIG_HPP_

#include <string>
#include <sstream>
#include <iniparser/iniparser.h>
#include "common.hpp"

#define FLYINGBERRY_CONFIG "flyingberry.cfg"

class FBConfig {
public:
    FBConfig();
    ~FBConfig();

    // General
    int looprate,default_mode;

    // Motor
    int esc_fl_pin,esc_fr_pin,esc_bl_pin,esc_br_pin;
    int esc_min_value,esc_max_value;

    // IMU
    int calibration_time,dlpf_level;
    float comp_filter_coefficient;

    // PID
    float stab_roll_pid[3],stab_pitch_pid[3],rate_roll_pid[3],rate_pitch_pid[3],yaw_pid[3];

    // Control
    int tcp_port;
};

extern FBConfig fbconfig;

#endif

