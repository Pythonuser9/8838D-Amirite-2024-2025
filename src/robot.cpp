#include "api.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"
using namespace pros;

#define LF_PORT 1 //correct
#define RF_PORT 4 //correct 
#define LM_PORT 2 //correct
#define RM_PORT 5 //correct
#define LB_PORT 3 //correct
#define RB_PORT 6 //correct    
#define IMU_PORT 21 // plugged in
#define INTAKE_PORT 8 //front roller
// #define HOOKS_PORT 8 //hooks 
#define LIFT_PORT 7 //lift
#define OPTICAL_PORT  // color switch, maybe prob not
#define ROTO_PORT 10



//#define
pros::Imu imu (IMU_PORT);
pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06,false);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06,true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06,true);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06,false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06,true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06,false);
// pros::Motor Intake(INTAKE_PORT, pros::E_MOTOR_GEARSET_06);
pros::Motor Intake(INTAKE_PORT, pros::E_MOTOR_GEARSET_06);
//pros::Optical Eyesight7 (OPTICAL_PORT);
pros::Motor LIFT(LIFT_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::ADIDigitalOut Mogo ('B', false);
// pros::ADIDigitalOut DaSorter ('B', false);
pros::ADIDigitalOut Doinker ('A', false);
pros::ADIDigitalIn selec ('C');
// pros::ADIDigitalOut Intakepiston ('C');
//pros::ADIAnalogIn DistanceSense ('A');
pros::Controller con (pros::E_CONTROLLER_MASTER);
pros::Rotation roto(ROTO_PORT);



