#include "api.h"
#include "main.h"
#include "okapi/api.hpp"


#ifndef PIDH
#define PIDH

extern float error;
extern int time2;
extern int automacro;
extern void liftauton();
extern double calcPID(double target, double input, int integralKI, int maxIntegral);
extern double calcPID2(double target, double input, int integralKI, int maxIntegral);
extern double calcPID3(double target, double input, int integralKI, int maxIntegral);
extern double calcPIDlift(double targetl, double inputl, int integralKIl, int maxIntegrall, int bias);
extern void setConstants(double kp, double ki, double kd);
extern void driveStraight(int target);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveStraight2(int target);
extern void driveStraightC(int target);
extern void driveClampSC(int target, int clampDistance, int speed);
extern void driveClamp(int target, int clampDistance);
extern void driveStraightSlow(int target, int speed);
extern void driveClampS(int target, int clampDistanceFromTarget, int speed);
extern void driveArcLS(double theta, double radius, int timeout, int speed);
extern void driveArcRS(double theta, double radius, int timeout, int speed);
extern void driveArcL(double theta, double radius, int timeout);
extern void driveArcR(double theta, double radius, int timeout);
extern void driveArcLF(double theta, double radius, int timeout);
extern void driveArcRF(double theta, double radius, int timeout);
//extern void ColorSort(int color);
extern void wallResetB(int resetTime);
extern void wallResetF(int resetTime);
extern void moveLift(int targetHeight);
extern void driveDoink(int target, int doinkDistance);


extern double totalError;

// straight stuff
#define STRAIGHT_KP 2.25 //3 
#define STRAIGHT_KI 0
#define STRAIGHT_KD 2.5
#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5

// turn stuff
#define TURN_KP 6
#define TURN_KI 0
#define TURN_KD 60
#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25


//arc turn stiff
//#define driveArcL 10
#define HEADING_KP 6
#define HEADING_KI 0
#define HEADING_KD 0
#define HEADING_MAX_INTEGRAL 0
#define HEADING_INTEGRAL_KI 0

#define ARC_HEADING_KP 4
#define ARC_HEADING_KI 0.01
#define ARC_HEADING_KD 1
#define ARC_HEADING_MAX_INTEGRAL 0
#define ARC_HEADING_INTEGRAL_KI 0

#define LIFT_KP 0.2
#define LIFT_KI 0
#define LIFT_KD 0

#define TOP_KP 0.1
#define TOP_KI 0
#define TOP_KD 1

#endif