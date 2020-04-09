
#ifndef _FBCONFIG_HPP_
#define _FBCONFIG_HPP_

#include <string>
#include <sstream>
#include <iniparser/iniparser.h>
#include "common.hpp"

#define FLYINGBERRY_CONFIG "flyingberry.cfg"

class FBConfig {
    void parse_float_values(const char*,float*);
    void parse_int_values(const char*,int*);

public:
    FBConfig();
    ~FBConfig();

    // Flight
    int looprate,default_mode;
    int stab_max_roll,stab_max_pitch;
    int min_base_throttle;
    int max_base_throttle;

    // Motor
    int esc_pin[MOTOR_TOTAL];
    int esc_min_value,esc_max_value;
    int max_throttle,max_throttle_increase;
    int delay_on;

    // IMU
    int calibration_time,dlpf_level;
    float comp_filter_coefficient;
    int accel_multipliers[3],gyro_multipliers[3];

    // PID
    float stab_roll_pid[3],stab_pitch_pid[3],rate_roll_pid[3],rate_pitch_pid[3],yaw_pid[3];
    int integral_limit,pid_limit;

    // Control
    
    // FPV
    int fpv_tcp_port,fpv_udp_port;
};

extern FBConfig fbconfig;

#endif

