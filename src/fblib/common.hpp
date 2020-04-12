
#ifndef _FBCOMMON_HPP_
#define _FBCOMMON_HPP_

#include <iostream> // XXX remove

// indexation
#define X 0 
#define Y 1 
#define Z 2 
#define ROLL 0
#define PITCH 1 
#define YAW 2
#define THROTTLE 3

enum class MotorPosition { front_left = 0, front_right = 1, back_left = 2, back_right = 3 };
enum class FlightMode { stabilize = 0, rate = 1 };
enum class PIDTerm { KP = 0, KI = 1, KD = 2 };
enum class ControllerMode { steam = 0, network = 1 };

typedef float Angle;

#endif

