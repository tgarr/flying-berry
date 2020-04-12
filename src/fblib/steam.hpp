
#ifndef _FBSTEAM_HPP_
#define _FBSTEAM_HPP_

#include "steamcontroller/steamcontroller.h"
#include "config.hpp"
#include "controller.hpp"

class SteamControllerHandler: public Controller {
    SteamControllerDevice* sc_device = NULL;
    SteamControllerDeviceEnum* sc_enum = NULL;
    SteamControllerEvent event;
    bool connected = false;
    float time_disconnected = 0;

    void connect();
    void disconnect();
    void connection_event(float);
    void input_event(float);

public:
    SteamControllerHandler();
    ~SteamControllerHandler();

    void update(float);
};

#endif

