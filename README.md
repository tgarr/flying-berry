# Flying Berry
Flying Berry is a DIY quadcopter drone powered by a Raspberry Pi.

This project includes a flight controller written in C++, intended to run on the
Rasperry Pi, as well as 3D models to print the whole frame.

**ATTENTION**: I gave up on this project and just built a proper FPV drone. The code probably works, but appropriate PID values must be found (basically through trial and error). I reused the code and components to build a RC Car powered by a Raspberry Pi, which I called RaspWheels: https://github.com/tgarr/raspwheels 

Features of the Flying Berry:
- Runs everything on the Raspberry Pi
- 3D printable frame
- Sensor fusion with a complementary filter
- Stabilize mode
- Controlled by a Steam Controller

