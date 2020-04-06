
#include "config.hpp"

FBConfig fbconfig;

void parse_float_values(const char *str,float* values){
    std::istringstream iss(str);
    iss >> values[0] >> values[1] >> values[2];
}

FBConfig::FBConfig(){
    dictionary* ini;
    ini = iniparser_load(FLYINGBERRY_CONFIG);

    // General
    looprate = iniparser_getint(ini,"General:looprate",-1);
    stab_max_roll = iniparser_getint(ini,"General:stab_max_roll",-1);
    stab_max_pitch = iniparser_getint(ini,"General:stab_max_pitch",-1);

    std::string mode(iniparser_getstring(ini,"General:default_mode",NULL));
    if(mode.compare("rate") == 0)
        default_mode = FB_FLIGHT_RATE;
    else
        default_mode = FB_FLIGHT_STABILIZE;

    // Motor
    esc_pin[FL] = iniparser_getint(ini,"Motor:esc_fl_pin",-1);
    esc_pin[FR] = iniparser_getint(ini,"Motor:esc_fr_pin",-1);
    esc_pin[BL] = iniparser_getint(ini,"Motor:esc_bl_pin",-1);
    esc_pin[BR] = iniparser_getint(ini,"Motor:esc_br_pin",-1);
    esc_min_value = iniparser_getint(ini,"Motor:esc_min_value",-1);
    esc_max_value = iniparser_getint(ini,"Motor:esc_max_value",-1);

    // IMU
    calibration_time = iniparser_getint(ini,"IMU:calibration_time",-1);
    dlpf_level = iniparser_getint(ini,"IMU:dlpf_level",-1);
    comp_filter_coefficient = iniparser_getdouble(ini,"IMU:comp_filter_coefficient",-1);

    // PID
    parse_float_values(iniparser_getstring(ini,"PID:stab_roll_pid",NULL),stab_roll_pid);
    parse_float_values(iniparser_getstring(ini,"PID:stab_pitch_pid",NULL),stab_pitch_pid);
    parse_float_values(iniparser_getstring(ini,"PID:rate_roll_pid",NULL),rate_roll_pid);
    parse_float_values(iniparser_getstring(ini,"PID:rate_pitch_pid",NULL),rate_pitch_pid);
    parse_float_values(iniparser_getstring(ini,"PID:yaw_pid",NULL),yaw_pid);

    // Control
    
    // FPV
    fpv_tcp_port = iniparser_getint(ini,"FPV:tcp_port",-1);
    fpv_udp_port = iniparser_getint(ini,"FPV:udp_port",-1);
  
    iniparser_freedict(ini);
}

FBConfig::~FBConfig(){
}

