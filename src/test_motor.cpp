
#include <iostream>
#include <string>
#include <unistd.h>
#include "fblib/motor.hpp"

void accel(Motor& m,float thr){
    std::cout << "Accelerating to " << thr*100 << "%" << std::endl;
    m.throttle(thr);
    sleep(2);
}

int main(int argc,char **argv) {
    int pin;

    if(argc < 2){
        std::cout << "usage: " << argv[0] << " <all|esc_gpio_pin> [throttle 1] [throttle 2]..." << std::endl;
        return 0;
    }

    bool all = false;
    if(std::string("all").compare(argv[1]) == 0)
        all = true;
    else
        pin = std::stoi(argv[1]);

    if(!fblib_initialise()){
        std::cerr << "Something went wrong! Try running as root." << std::endl;
        return 0;
    }
    
    std::cout << "Connect the battery and press enter";
    std::cin.ignore();
    sleep(2);  

    if(all){
        Motor fl(MotorPosition::front_left);
        Motor fr(MotorPosition::front_right);
        Motor bl(MotorPosition::back_left);
        Motor br(MotorPosition::back_right);

        all_motors_on(fl,fr,bl,br);
        
        for(int i=2;i<argc;i++){
            float thr = std::stof(argv[i]);

            std::cout << "Front left: "; accel(fl,thr);
            std::cout << "Front right: "; accel(fr,thr);
            std::cout << "Back left: "; accel(bl,thr);
            std::cout << "Back right: "; accel(br,thr);
        }

        std::cout << "Turning off" << std::endl;
        fl.off();
        fr.off();
        bl.off();
        br.off();
    }
    else {
        Motor motor(pin);
        motor.on();

        for(int i=2;i<argc;i++){
            float thr = std::stof(argv[i]);
            accel(motor,thr);
        }

        std::cout << "Turning off" << std::endl;
        motor.off();
    }

    sleep(2);
    fblib_finalize();

    return 0;
}

