
#include <iostream>
#include <string>
#include <pigpio.h>
#include <unistd.h>
#include "fblib/motor.hpp"

int main(int argc,char **argv) {
    int pin;

    if(argc < 2){
        std::cout << "usage: " << argv[0] << " <esc_gpio_pin> [throttle]" << std::endl;
        return 0;
    }

    pin = std::stoi(argv[1]);

    if(gpioInitialise() == PI_INIT_FAILED){
        std::cerr << "Something went wrong! Try running as root." << std::endl;
        return 0;
    }

    Motor motor(pin);

    std::cout << "Connect the battery and press enter";
    std::cin.ignore();
  
    sleep(2);  
    motor.on();

    if(argc > 2){
        float thr = std::stof(argv[2]);
        std::cout << "Accelerating to " << thr*100 << "%" << std::endl;
        motor.throttle(thr);
        sleep(10);
    }
    else {
        std::cout << "Accelerating to 30%" << std::endl;
        motor.throttle(0.3);
        sleep(2);
    
        std::cout << "Accelerating to 50%" << std::endl;
        motor.throttle(0.5);
        sleep(2);
    
        std::cout << "Accelerating to 70%" << std::endl;
        motor.throttle(0.7);
        sleep(2);
    
        std::cout << "Accelerating to 100%" << std::endl;
        motor.throttle(1);
        sleep(2);
    
        std::cout << "Decelerating to 50%" << std::endl;
        motor.throttle(0.5);
        sleep(2);
    
        std::cout << "Decelerating to 20%" << std::endl;
        motor.throttle(0.2);
        sleep(2);
    }

    std::cout << "Turning off" << std::endl;
    motor.off();
    sleep(2);

    gpioTerminate();

    return 0;
}

