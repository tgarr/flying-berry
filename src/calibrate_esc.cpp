
#include <iostream>
#include <string>
#include <pigpio.h>
#include <unistd.h>
#include "fblib/motor.hpp"

int main(int argc,char **argv) {
    int pin;

    if(argc < 2){
        std::cout << "usage: " << argv[0] << " <esc_gpio_pin>" << std::endl;
        return 0;
    }

    pin = std::stoi(argv[1]);

    if(gpioInitialise() == PI_INIT_FAILED){
        std::cerr << "Something went wrong! Try running as root." << std::endl;
        return 0;
    }

    Motor motor(pin);

    std::cout << "Disconnect the battery from the ESC and press enter";
    std::cin.ignore();

    motor.start_calibration();
    motor.calibration_max(); // max throttle
    std::cout << "Connect the battery and press enter";
    std::cin.ignore();

    sleep(2);
    motor.calibration_min(); // min throttle
    sleep(2);
    motor.stop_calibration();

    gpioTerminate();

    std::cout << "Calibration complete!" << std::endl;
    return 0;
}


