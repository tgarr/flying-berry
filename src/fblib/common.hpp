
#ifndef _FBCOMMON_HPP_
#define _FBCOMMON_HPP_

#include <iostream> // XXX remove

// indexation
#define X 0 // x axis
#define Y 1 // y axis
#define Z 2 // z axis
#define ROLL 0 // roll angle
#define PITCH 1 // pitch angle
#define YAW 2 // yaw angle

enum class MotorPosition { front_left = 0, front_right = 1, back_left = 2, back_right = 3 };
enum class FlightMode { stabilize = 0, rate = 1 };
enum class PIDTerm { KP = 0, KI = 1, KD = 2 };

typedef float Angle;

#endif

