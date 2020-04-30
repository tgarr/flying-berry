
#include "config.hpp"

FBConfig fbconfig;

void FBConfig::parse_float_values(const char *str,float* values,int num){
    std::istringstream iss(str);
    for(int i=0;i<num;i++) iss >> values[i];
}

void FBConfig::parse_int_values(const char *str,int* values,int num){
    std::istringstream iss(str);
    for(int i=0;i<num;i++) iss >> values[i];
}

FBConfig::FBConfig(){
    dictionary* ini;
    ini = iniparser_load(FLYINGBERRY_CONFIG);

    // Flight
    looprate = iniparser_getint(ini,"Flight:looprate",-1);
    if(looprate > FB_MAX_LOOPRATE) looprate = FB_MAX_LOOPRATE;

    stab_max_roll = iniparser_getint(ini,"Flight:stab_max_roll",-1);
    stab_max_pitch = iniparser_getint(ini,"Flight:stab_max_pitch",-1);
    min_base_throttle = iniparser_getint(ini,"Flight:min_base_throttle",-1) / 100.0f;
    max_base_throttle = iniparser_getint(ini,"Flight:max_base_throttle",-1) / 100.0f;

    std::string mode(iniparser_getstring(ini,"Flight:default_mode",NULL));
    if(mode.compare("rate") == 0)
        default_mode = FlightMode::rate;
    else
        default_mode = FlightMode::stabilize;

    // Motor
    parse_int_values(iniparser_getstring(ini,"Motor:esc_pins",NULL),esc_pin,4);
    esc_min_value = iniparser_getint(ini,"Motor:esc_min_value",-1);
    esc_max_value = iniparser_getint(ini,"Motor:esc_max_value",-1);
    max_throttle = iniparser_getint(ini,"Motor:max_throttle",-1) / 100.0f;
    max_throttle_increase = iniparser_getint(ini,"Motor:max_throttle_increase",-1) / 100.0f;
    delay_on = iniparser_getint(ini,"Motor:delay_on",-1);

    // IMU
    calibration_time = iniparser_getint(ini,"IMU:calibration_time",-1);
    dlpf_level = iniparser_getint(ini,"IMU:dlpf_level",-1);
    comp_filter_coefficient = iniparser_getdouble(ini,"IMU:comp_filter_coefficient",-1);
    parse_int_values(iniparser_getstring(ini,"IMU:accel_multipliers",NULL),accel_multipliers,3);
    parse_int_values(iniparser_getstring(ini,"IMU:gyro_multipliers",NULL),gyro_multipliers,3);

    // PID
    parse_float_values(iniparser_getstring(ini,"PID:stab_roll_pid",NULL),pid[static_cast<int>(FlightMode::stabilize)][ROLL],3);
    parse_float_values(iniparser_getstring(ini,"PID:stab_pitch_pid",NULL),pid[static_cast<int>(FlightMode::stabilize)][PITCH],3);
    parse_float_values(iniparser_getstring(ini,"PID:yaw_pid",NULL),pid[static_cast<int>(FlightMode::stabilize)][YAW],3);
    parse_float_values(iniparser_getstring(ini,"PID:rate_roll_pid",NULL),pid[static_cast<int>(FlightMode::rate)][ROLL],3);
    parse_float_values(iniparser_getstring(ini,"PID:rate_pitch_pid",NULL),pid[static_cast<int>(FlightMode::rate)][PITCH],3);
    parse_float_values(iniparser_getstring(ini,"PID:yaw_pid",NULL),pid[static_cast<int>(FlightMode::rate)][YAW],3);
    integral_limit = iniparser_getint(ini,"PID:integral_limit",-1);
    pid_limit = iniparser_getint(ini,"PID:pid_limit",-1);
    pid_multiplier = iniparser_getdouble(ini,"PID:pid_multiplier",-1);

    // Controller
    disconnected_time_limit = iniparser_getint(ini,"Controller:disconnected_time_limit",-1);
    mode = iniparser_getstring(ini,"Controller:default",NULL);
    if(mode.compare("network") == 0)
        default_controller = ControllerMode::network;
    else
        default_controller = ControllerMode::steam;

    // Steam Controller
    roll_sensitivity = iniparser_getint(ini,"Steam_Controller:roll_sensitivity",-1) / 100.0f;
    pitch_sensitivity = iniparser_getint(ini,"Steam_Controller:pitch_sensitivity",-1) / 100.0f;
    yaw_sensitivity = iniparser_getint(ini,"Steam_Controller:yaw_sensitivity",-1) / 100.0f;
    throttle_sensitivity = iniparser_getint(ini,"Steam_Controller:throttle_sensitivity",-1) / 100.0f;
    
    // FPV
    fpv_tcp_port = iniparser_getint(ini,"FPV:tcp_port",-1);
    fpv_udp_port = iniparser_getint(ini,"FPV:udp_port",-1);
  
    iniparser_freedict(ini);
}

FBConfig::~FBConfig(){
}

