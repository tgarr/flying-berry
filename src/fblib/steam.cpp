
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
                SteamController_Configure(dev, STEAMCONTROLLER_DEFAULT_FLAGS|STEAMCONTROLLER_DEFAULT_TIMEOUT);
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
    last_attempt = 0;
}

void SteamControllerHandler::input_event(float dt){
    // buttons
    if(event.update.buttons != 0){
        // start
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_NEXT){
            drone->start();
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

        // stop
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_PREV){
            drone->stop();
        }

        // burst
        if(event.update.buttons & STEAMCONTROLLER_BUTTON_Y){
            float t = 3 * dt * fbconfig.throttle_sensitivity;
            drone->throttle(drone->throttle() + t);
        }
        else if(event.update.buttons & STEAMCONTROLLER_BUTTON_A){
            float t = 3 * dt * fbconfig.throttle_sensitivity;
            drone->throttle(drone->throttle() - t);
        }
    }

    // stabilize mode
    // TODO rate mode
    float roll,pitch;
    if((event.update.rightXY.x == 0) || (event.update.rightXY.y == 0)){
        roll = 1 * event.update.rightXY.x / (float)STEAMCONTROLLER_RIGHTPAD_RADIUS * 100 * fbconfig.roll_sensitivity;
        pitch = -1 * event.update.rightXY.y / (float)STEAMCONTROLLER_RIGHTPAD_RADIUS * 100 * fbconfig.pitch_sensitivity;
    }
    else {
        float x = event.update.rightXY.x;
        float y = event.update.rightXY.y;
        float d = atan(abs(y)/abs(x));
        float c = sqrt(x*x + y*y);
        float crate = c / STEAMCONTROLLER_RIGHTPAD_RADIUS;

        roll = 1 * copysign(crate * ((M_PI/2-d)/(M_PI/2)),x) * 100 * fbconfig.roll_sensitivity;
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
        last_attempt += dt;

        if(time_disconnected >= fbconfig.disconnected_time_limit){
            if(last_attempt >= 1){ // XXX once per second
                drone->throttle(drone->throttle() * 0.97);
                last_attempt = 0;
            }

            // TODO drone->descend();
        }

        // try to connect only if not flying
        if(!drone->is_flying()) connect();
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

