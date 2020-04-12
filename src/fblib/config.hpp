
#ifndef _FBCONFIG_HPP_
#define _FBCONFIG_HPP_

#include <string>
#include <sstream>
#include <iniparser/iniparser.h>
#include "common.hpp"

#define FLYINGBERRY_CONFIG "flyingberry.cfg"

class FBConfig {
    void parse_float_values(const char*,float*,int);
    void parse_int_values(const char*,int*,int);

public:
    FBConfig();
    ~FBConfig();

    // Flight
    int looprate;
    FlightMode default_mode;
    int stab_max_roll,stab_max_pitch;
    float min_base_throttle;
    float max_base_throttle;

    // Motor
    int esc_pin[4];
    int esc_min_value,esc_max_value;
    float max_throttle,max_throttle_increase;
    int delay_on;

    // IMU
    int calibration_time,dlpf_level;
    float comp_filter_coefficient;
    int accel_multipliers[3],gyro_multipliers[3];

    // PID
    float pid[2][3][3];
    int integral_limit,pid_limit;

    // Controller
    ControllerMode default_controller;
    int disconnected_time_limit;

    // Steam Controller
    float roll_sensitivity,pitch_sensitivity,yaw_sensitivity,throttle_sensitivity;
    
    // FPV
    int fpv_tcp_port,fpv_udp_port;
};

extern FBConfig fbconfig;

#endif

