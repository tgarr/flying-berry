
#ifndef _FBCOMMON_HPP_
#define _FBCOMMON_HPP_

#include <iostream> // XXX remove

#define FB_FLIGHT_STABILIZE 0 // stabilize flight mode
#define FB_FLIGHT_RATE 1 // rate flight mode

// indexation
#define DIM 3 // dimensions
#define X 0 // x axis
#define Y 1 // y axis
#define Z 2 // z axis
#define ROLL 0 // roll angle
#define PITCH 1 // pitch angle
#define YAW 2 // yaw angle
#define KP 0 // proportional gain
#define KI 1 // integral gain
#define KD 2 // derivative gain
#define MOTOR_TOTAL 4 // quadcopter
#define FL 0 // front left motor
#define FR 1 // front right motor
#define BL 2 // back left motor
#define BR 3 // back right motor

typedef float Angle;

#endif

