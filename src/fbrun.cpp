
#include <iostream>
#include "fblib/drone.hpp"

int main() {
    Drone drone;

    if(!drone.setup()){
        std::cerr << "[ERROR] Something went wrong! Try running as root." << std::endl;
        return 0;
    }

    drone.run();
    drone.finalize();
    return 0;
}

