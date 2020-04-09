
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
    int stab_max_roll,stab_max_pitch;
    int start_throttle;

    // Motor
    int esc_pin[MOTOR_TOTAL];
    int esc_min_value,esc_max_value;
    int max_throttle,max_throttle_increase;

    // IMU
    int calibration_time,dlpf_level;
    float comp_filter_coefficient;

    // PID
    float stab_roll_pid[3],stab_pitch_pid[3],rate_roll_pid[3],rate_pitch_pid[3],yaw_pid[3];
    int integral_limit,pid_limit;

    // Control
    int max_base_throttle;
    
    // FPV
    int fpv_tcp_port,fpv_udp_port;
};

extern FBConfig fbconfig;

#endif

