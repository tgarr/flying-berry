
#include "steam.hpp"

SteamControllerHandler::SteamControllerHandler(){
    connected = false;
    time_disconnected = 0;
}

SteamControllerHandler::~SteamControllerHandler(){
    disconnect();
}

void SteamControllerHandler::connect(){
    sc_enum = SteamController_EnumControllerDevices();
    while(sc_enum){
        SteamControllerEvent event;
        SteamControllerDevice* dev = SteamController_Open(sc_enum);
        if(dev){
            if(SteamController_ReadEvent(dev,&event) == STEAMCONTROLLER_EVENT_CONNECTION && event.connection.details == STEAMCONTROLLER_CONNECTION_EVENT_CONNECTED){
                SteamController_Configure(dev, STEAMCONTROLLER_DEFAULT_FLAGS);
                sc_device = dev;
                connected = true;
                time_disconnected = 0;
                SteamController_PlayMelody(sc_device,0x04);
                return;
            }
        }
        SteamController_Close(dev);
        sc_enum = SteamController_NextControllerDevice(sc_enum);
    }

    sc_device = NULL;
    connected = false;
}

void SteamControllerHandler::disconnect(){
    SteamController_Close(sc_device);
    connected = false;
}

void SteamControllerHandler::input_event(float dt){
    // buttons
    if(event.update.buttons != 0){
        // take-off
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_NEXT){
            drone->takeoff();
        }

        // throttle rear triggers
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_LT){
            float t = dt * fbconfig.throttle_sensitivity;
            drone->throttle(drone->throttle() - t);
        }
        else if(event.update.buttons & STEAMCONTROLLER_BUTTON_RT){
            float t = dt * fbconfig.throttle_sensitivity;
            drone->throttle(drone->throttle() + t);
        }

        // panic
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_X){
            drone->panic();
        }

        // off
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_PREV){
            drone->stop(); // TODO land
        }
    }

    // stabilize mode
    // TODO rate mode
    float roll,pitch;
    if((event.update.rightXY.x == 0) || (event.update.rightXY.y == 0)){
        roll = event.update.rightXY.x / (float)STEAMCONTROLLER_RIGHTPAD_RADIUS * 100 * fbconfig.roll_sensitivity;
        pitch = -1*event.update.rightXY.y / (float)STEAMCONTROLLER_RIGHTPAD_RADIUS * 100 * fbconfig.pitch_sensitivity;
    }
    else {
        float x = event.update.rightXY.x;
        float y = event.update.rightXY.y;
        float d = atan(abs(y)/abs(x));
        float c = sqrt(x*x + y*y);
        float crate = c / STEAMCONTROLLER_RIGHTPAD_RADIUS;

        roll = copysign(crate * ((M_PI/2-d)/(M_PI/2)),x) * 100 * fbconfig.roll_sensitivity;
        pitch = -1 * copysign(crate * (d/(M_PI/2)),y) * 100 * fbconfig.pitch_sensitivity;
    }

    drone->roll(roll);
    drone->pitch(pitch);
    // TODO yaw
}

void SteamControllerHandler::connection_event(float dt){
    switch(event.connection.details){
        case STEAMCONTROLLER_CONNECTION_EVENT_CONNECTED:
            connected = true;
            time_disconnected = 0;
            std::cout << "Steam Controller connected!" << std::endl;
            break;
        case STEAMCONTROLLER_CONNECTION_EVENT_DISCONNECTED:
            std::cout << "Steam Controller disconnected!" << std::endl;
            disconnect();
            drone->panic();
            break;
    }
}

void SteamControllerHandler::update(float dt){
    if(!connected){
        time_disconnected += dt;
        if(time_disconnected >= fbconfig.disconnected_time_limit){
            float t = dt * fbconfig.throttle_sensitivity;
            drone->throttle(drone->throttle() - t);
        }

        connect();
        return;
    }

    switch(SteamController_ReadEvent(sc_device,&event)){
        case STEAMCONTROLLER_EVENT_UPDATE:
            input_event(dt);
            break;
        case STEAMCONTROLLER_EVENT_CONNECTION:
            connection_event(dt);
            break;
    }
}

