
#include <iostream>
#include "fblib/drone.hpp"
#include "fblib/steam.hpp"

int main() {
    // TODO get default controller from config file
    Controller* controller = new SteamControllerHandler; 
    Drone drone(controller);

    if(!drone.setup()){
        std::cerr << "Something went wrong! Try running as root." << std::endl;
        return 0;
    }

    drone.main();
    drone.finalize();
    return 0;
}

