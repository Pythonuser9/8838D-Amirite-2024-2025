#include "main.h"
#include "robot.h"
#include "api.h"
#include "pid.h"
#include "auton.h"
#include "odometry.h"
//#include "odometry.h"
using namespace std;
using namespace pros;


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */




void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "It's go time!! ");

	pros::lcd::register_btn1_cb(on_center_button);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
int atn = 0;
int pressed = 0;
string autstr;


void competition_initialize() {
while(true){
	int selecs = 0;
	if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
		selecs++;
		delay(200);
		if(selecs >= 3){
		 selecs == 0;
		}
	}
//auto selector below 
//comment
	if(selec.get_value() == true){
		pressed++;
	}

	if (selec.get_value() == false){
		pressed = 0;
	}

	if(pressed == 1){
		atn++;
	}

	//autons here ===>

	if(atn == 0) {
		autstr = "RedLeft";
		con.print(0,0, "aut 0: %s", autstr);
	}
	else if(atn == 1) {
		autstr = "BlueRight";
		con.print(0,0, "aut 1: %s", autstr);
	}
	else if(atn == 2) {
		autstr = "RedRight";
		con.print(0,0, "aut 2: %s", autstr);
	}
	else if(atn == 3) {
		autstr = "BlueLeft";
		con.print(0,0, "aut 3: %s", autstr);
	}
	else if(atn == 4) {
		autstr = "Sig Awp Red";
		con.print(0,0, "aut 4: %s", autstr);
	}
	else if(atn == 5) {
		autstr = "Sig Awp Blue";
		con.print(0,0, "aut 5: %s", autstr);
	}
	else if(atn == 6) {
		autstr = "Skills";
		con.print(0,0, "aut 6: %s", autstr);
	}
	else if(atn == 7) {
		autstr = "Safety";
		con.print(0,0, "aut 7: %s", autstr);
		atn = 0;
	}
	// else if(atn == 8) {
	// 	autstr = "Skills";
	// 	con.print(0,0, "aut 8: %s", autstr);
	// }
	// else if(atn == 9) {
	// 	autstr = "Safety";
	// 	con.print(0,0, "aut 9: %s", autstr);
	// }
	// else if(atn == 10){
	// 	atn = 0;
	// }

	con.clear();}
	
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */




void opcontrol() {
	LF.set_brake_mode(E_MOTOR_BRAKE_COAST);
	bool arcToggle = true;
	bool tankToggle = false;
	bool PistonsForMogo = false;
	int lift_macro = 0;
	bool lift_toggle = false;
	bool doinker = false;
	// bool intakepiston = true;
	bool NEWR1 = false;
	bool NEWR2 = false;
	double lift_angle = 0;
	double rotoangle = 0;
	int time = 0;
	string red;
	string blue;
	
	int color_selec = 1;
	//Eyesight.set_led_pwm(100);
while (true){
	   





	if (con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
		arcToggle = !arcToggle;
		tankToggle = !tankToggle;
	}

	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
		NEWR1 = true;
	} else {
		NEWR1 = false;
	}

	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
		NEWR2 = true;
	} else {
		NEWR2 = false;
	}

// if (con.get_digital(E_CONTROLLER_DIGITAL_LEFT)) {
// 	Intakepiston.set_value(true);
// } else {
// 	Intakepiston.set_value(false);
// }

	if (tankToggle){
 LF.move(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
 LM.move(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
 LB.move(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
 RF.move(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
 RM.move(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
 RB.move(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
	}
	if (arcToggle){
  int RX = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
  int power = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = int(pow(RX, 5)/ pow(127, 4));
  int left = power + turn;
  int right = power - turn;

 LF.move(left);
 LM.move(left);
 LB.move(left);
 RF.move(right);
 RM.move(right);
 RB.move(right);
}
//auton selector
if(atn == 0) {
		autstr = "Red ring";
	}
	if(atn == 1) {
		autstr = "Blue ring";
	}
	if(atn == 2) {
		autstr = "Blue mogo";
	}
	if(atn == 3) {
		autstr = "Red mogo";
	}
	if(atn == 4) {
		autstr = "RedLeftElims";
	}
	if(atn == 5) {
		autstr = "BlueRightElims";
	}
	if(atn == 6) {
		autstr = "RedRightElims";
	}
	if(atn == 7) {
		autstr = "BlueLeftElims";
	}
	if(atn == 8) {
		autstr = "Skills";
	}
	if(atn == 9) {
		autstr = "Safety";
	}
	else if(atn == 10){
		atn = 0;
	}

	

	if(selec.get_value() == true){
		pressed++;
	}

	if (selec.get_value() == false){
		pressed = 0;
	}

	if(pressed == 1){
		atn++;
	}
////////////////////////////////////////////////change values
//da

// if (con.get_digital(E_CONTROLLER_DIGITAL_X)) { 
// //	driveTurn2(175);
// driveTurn2(100);
// driveTurn2(-170);
// //driveArcL(90, 700, 30000);
// }
	
//////////////////////////////////////////ADD THIS FOR COLOR TESTING 
	//ColorSort(color_selec);

/////////////////////////////////////////////
	
	// if (((con.get_digital(E_CONTROLLER_DIGITAL_R1) && NEWR2) || (con.get_digital(E_CONTROLLER_DIGITAL_R2) && NEWR1)) || (NEWR1 && NEWR2)){
	// lift_macro = 0;
	// } else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
	// 	Intake.move(-127);
	// } else if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
	// 	Intake.move(127);
	// }
	// else  {
	// 	Intake.move(0);
	// }


	if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
	Intake.move(-127);
	} else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
	Intake.move(127);
	} else {
	Intake.move(0);
	}


if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){


lift_macro = 0;

}


if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){

	lift_macro = 0;

}

if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
	PistonsForMogo = !PistonsForMogo;
}
Mogo.set_value(PistonsForMogo);
// ////////////////////////////////////////////////////////////// make this a piston flip out 
if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
	doinker = !doinker;
}
Doinker.set_value(doinker);

// if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
// 	intakepiston = !intakepiston;
// }
// Intakepiston.set_value(intakepiston);




//



// 	if (roto.get_position() < 300) {
// 	LIFT.move(127);
// 	lift_angle = LIFT.get_position();
// 	lift_toggle = false;
// 	} else {
// 	setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
// 	LIFT.move(calcPID(lift_angle, LIFT.get_position(), 0, 0));
// 	}}









if (con.get_digital(E_CONTROLLER_DIGITAL_L1)){
	LIFT.move(127);
	lift_angle = LIFT.get_position();
	lift_toggle = false;
} else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)){
	LIFT.move(-127);
	lift_angle = LIFT.get_position();
	lift_toggle = false;
} else {
	setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
	LIFT.move(calcPID(lift_angle, LIFT.get_position(), 0, 0));
} 



// if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
// while (roto.get_position() > LIFT_TARGET_HEIGHT){
// 	LIFT.move(-127);
// 	lift_angle = LIFT.get_position();
// 	lift_toggle = false;
// 	delay(20);
// } 	setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
// 	LIFT.move(calcPID(lift_angle, LIFT.get_position(), 0, 0));
// }









// if(con.get_digital(E_CONTROLLER_DIGITAL_L2)){
// 	LIFT.move(-127);
// 	lift_toggle = false;
// } else if (con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
// 	LIFT.move(127);
// 	lift_toggle = false;
// } else {
// 	LIFT.move(0);
// 	LIFT.brake();
// 	}


if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
	lift_macro++;
	lift_toggle = true;
}

rotoangle = roto.get_angle();
if (rotoangle > 33000){
	rotoangle = rotoangle - 36000;
}

if (lift_toggle){
	if (lift_macro == 0) {
		setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
 		LIFT.move(calcPIDlift(500, roto.get_angle(), 0, 0, 3));
	} else if (lift_macro == 1) {
		setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
 		LIFT.move(calcPIDlift(3850, roto.get_angle(), .01, 1, 3));
	} else if (lift_macro == 2){
        setConstants(TOP_KP, TOP_KI, TOP_KD);
 		LIFT.move(calcPIDlift(15000, roto.get_angle(), 0, 0, 1));
    } else {
		lift_macro = 0;
	}
}

// else if (lift_macro == 2) {
// 		setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
//  		LIFT.move(calcPIDlift(5300, roto.get_angle(), 0, 0, 3));
// 	} 

pros::delay(1);
time += 1;

if (time % 50 == 0 && time % 100 !=0 && time % 150 != 0){
con.print(0, 0, "Auton: %s			", autstr);
} else if (time % 100 == 0 && time % 150 != 0){
con.print(1, 0, "ERROR %f 			", float (error));
} else if (time % 150 == 0){
	con.print(2, 0, " Time: %f 			", float (time2));
}

}



// if (con.get_digital(E_CONTROLLER_DIGITAL_X)){
// 	driveArcL(90, 1000, 10000);
// }
}














