#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"
using namespace std;
using namespace pros;


void autonomous() {


    
    if(atn == 0){    
    //Red Ring
    
    Intake.move(127);
    Mogo.set_value(true);
    driveStraightC(925);
    driveArcL(45, 400, 900);
    driveStraightSlow(200, 80);
    delay(100);
    Intake.move(0);
    driveTurn2(-54);
    driveClampS(-825, 50, 75);
    delay(100);
    Intake.move(127);
    driveTurn2(-60);
    driveStraight2(1250);
    delay(600);
    driveStraight2(-100);
    driveStraightC(-450);
    driveTurn2(-90);
    delay(200);
    driveStraightC(150);
    driveArcLF(95, 450, 1000);
    driveStraightC(650);
    driveArcRF(20, 200, 1000);
    driveTurn2(-135);
    driveStraightSlow(175, 80);
    delay(300);
    driveStraight2(-400);
    driveTurn2(106);
    driveStraight2(700);
    driveTurn2(47.5);
    driveStraight2(1200);
    driveStraightSlow(200, 60);







   

    

    }
    if(atn == 1) {
      //Blue Ring
     
    Intake.move(127);
    Mogo.set_value(true);
    driveStraightC(925);
    driveArcR(45, 400, 900);
    driveStraightSlow(200, 80);
    delay(100);
    Intake.move(0);
    driveClampS(-825, 50, 70);
    delay(100);
    Intake.move(127);
    driveTurn2(60);
    driveStraight2(1050);
    delay(600);
    driveStraightC(-550);
    driveTurn2(90);
    delay(200);
    driveStraightC(300);
    driveArcRF(90, 400, 1000);
    driveStraightC(200);
    driveArcLF(30, 150, 1000);
    driveTurn2(133);
    driveStraight2(200);
    driveStraightSlow(500, 80);
    delay(200);
    driveStraightSlow(-250, 65);
    driveTurn2(-100);
    driveStraightSlow(750, 70);
    driveTurn2(-40);
    driveStraight2(1000);
    driveStraightSlow(300, 80);

      ///////////////
     
      



      
      //driveArcL(-40, 850, 800);
      //delay(300);
      //driveTurn2(20);
      //driveStraightC(-1000);
      // Mogo.set_value(true);

 
  
    // blue ring rush
    // Mogo.set_value(true);
    // Intake.move(127);
    // driveStraightC(900);
    // driveArcR(50, 320, 1000);
    // driveStraight2(230);
    // Intake.move(0);
    // delay(300);
    // // driveTurn2(50);
    // driveClampS(-875, 45, 40);
    // Intake.move(127);
    // driveTurn2(90);
    // driveStraight2(700);
    // delay(100);
    // driveStraight2(375);
    // driveTurn2(5);
    // driveStraightSlow(540, 55);
    // driveStraight2(-500);
    // driveTurn2(180);
    // driveStraight2(1300);
    // delay(250);
    // driveStraight2(-250);






    }

    if(atn == 2){ 
    //blue mogo
    Mogo.set_value(true);
    driveStraightC(1275);
    driveTurn2(25);
    Doinker.set_value(true);
    driveArcRS(-50, 1700, 1500, 80);
    Doinker.set_value(false);
    driveStraight2(-150);
    driveTurn2(175);
    delay(200);
    driveClampS(-600, 50, 70);
    Intake.move(127);
    driveStraightC(450);
    driveTurn2(-135);
    driveStraightSlow(450, 80);
    delay(250);
    driveStraightSlow(200, 60);
    delay(100);
    driveStraightC(-450);
    driveTurn2(0);
    delay(200);
    Intake.move(-10);
    Mogo.set_value(true);
  Intake.move(127);
    driveStraight2(675);
    delay(50); 
    Intake.move(0);
    driveTurn2(-90);
    driveClampS(-800, 50, 80);
    Intake.move(127);
    driveTurn2(225);
    driveStraight2(700);
    
   //
    }



    if(atn == 3){
    //red mogo
    Mogo.set_value(true);
    driveStraightC(1225);
    driveTurn2(30);
    Doinker.set_value(true);
    delay(200);
    driveTurn2-(55);
    driveStraight2(-300);
    Doinker.set_value(false);
    driveTurn2(45);
    driveClampS(-600, 50, 85);
    driveTurn2(90);
    Intake.move(127);
    driveStraightSlow(725, 70);
    delay(700);
    driveStraightC(-300);
    delay(200);
    Mogo.set_value(true);
    driveStraightC(400);
    driveTurn2(155);
    delay(400);
    driveClampS(-500, 50, 65);
    driveStraight2(2200);
    driveTurn2(135);
    driveStraightSlow(350, 65);
    delay(300);
    driveStraightSlow(-300, 55);
    driveStraight2(-300);
    Doinker.set_value(true);
    driveStraight2(400);
    driveTurn2(-360);
 
  
    // driveTurn2(-175);
    // delay(200);
    // driveClampS(-600, 50, 70);
    // Intake.move(127);
    // driveStraightC(500);
    // driveTurn2(135);
    // driveStraightSlow(400, 80);
    // delay(150);
    // driveStraightC(-450);
    // driveTurn2(0);
    // Mogo.set_value(true);
    // driveStraight2(550);
    // delay(100); 
    // Intake.move(0);
    // driveTurn2(90);
    // driveClampS(-800, 50, 80);
    // Intake.move(127);
    // driveTurn2(-90);
    // driveStraight2(900);
    // driveStraightSlow(200, 80);
    }



    if(atn == 4){
      //sigawp red
      Mogo.set_value(true);
      Intake.move(127);
      driveStraightC(925);
      driveArcL(45, 420, 900);
      driveStraightSlow(170, 80);
      delay(100);
      Intake.move(20);
      driveTurn2(-54);
      delay(50);
      driveClampS(-775, 85, 93); //1st mogo
      Intake.move(127);
      driveTurn2(-90);
      driveStraightC(500);
      driveStraightSlow(400, 70); //first ring
      Intake.move(-10);
      driveTurn2(135);
      Intake.move(127);
      driveStraightC(400); //2nd ring
      Mogo.set_value(true);
      driveStraightC(1050);
      driveTurn2(177);
      LIFT.move(127); //alliance stake
      driveStraightSlow(330, 50);
      delay(50);
      driveStraight2(60);
      LIFT.move(-127);
      driveTurn2(180);
      driveStraightC(-325);
      driveTurn2(-132.5);
      driveStraightC(-800);
      driveClampS(-480, 80, 80);
      LIFT.move(0);
      driveTurn2(93);
      Intake.move(127);
      driveStraightC(600);
      driveStraightSlow(300, 45);
      LIFT.move(127);
      driveStraightC(-800);
      driveTurn2(-37);
      driveStraightSlow(270, 90);
      //driveTurn2(-65);







    }



    if(atn == 5) {
      //blue sigawp
      
      Intake.move(127);
      Mogo.set_value(true);
      driveStraightC(875);
      driveArcR(45, 400, 900);
      driveStraightSlow(160, 80);
      delay(100);
      Intake.move(20);
      driveClampS(-825, 50, 70); //1st mogo
      delay(100);
      Intake.move(127); //first ring
      driveTurn2(90);
      driveStraightC(500);
      driveStraightSlow(400, 70); //2nd ring
      Intake.move(-5);
      driveTurn2(-126);
      Intake.move(127);
      driveStraightC(400);
      Mogo.set_value(true);
      Intake.move(-127);
      driveStraightC(1800);
      driveStraightC(-150); //to alliance stake
      driveTurn2(-180);
      LIFT.move(127); //alliance stake
      Intake.move(127);
      delay(150);
      driveStraightSlow(600, 60);
      Intake.move(-127);
      LIFT.move(-127);
      driveTurn2(-180);
      driveStraightC(-325);
      Intake.move(127);
      driveTurn2(125); //turn for mogo
      driveStraightC(-800);
      driveClampS(-520, 80, 80); //2nd mogo
      Intake.move(127);
      LIFT.move(0);
      driveTurn2(-93); //start to get 3rd ring
      Intake.move(200);
      driveStraightC(600);
      driveStraightSlow(300, 45);
      LIFT.move(127); //ladybrown up
      driveStraightC(-800);
      driveTurn2(37);
      driveStraightSlow(270, 90); //hang
    }
    


    if(atn == 6) {
     //skills   
    Mogo.set_value(true);
    LIFT.move(127);
    delay(400);
    driveStraight2(150);
    LIFT.move(-127); //ladybrown down
    delay(200);
    Intake.move(127);
    driveStraightC(-275); 
    driveTurn2(-90);
    driveClampS(-1100, 50, 80); //mogo clamp
    LIFT.move(0);
    driveTurn2(180);
    driveStraight2(800);
    driveTurn2(135);
    driveStraight2(1700);
    Intake.move(0);
    driveTurn2(90);
    driveStraight2(100);
    LIFT.move(100);
    Intake.move(105);
    delay(160);
    LIFT.move(10);
    LIFT.set_brake_mode(E_MOTOR_BRAKE_COAST);
    driveStraight2(170);
    delay(180);
    Intake.move(-15);
    LIFT.move(127);
    driveStraight2(300);
    delay(650);
    driveStraight2(-400);
    LIFT.move(-127);
    driveTurn2(-2);
    Intake.move(127);
    driveStraight2(1100);
    driveStraightSlow(925, 50);
    delay(200);
  
    // driveTurn2(118.5);
    // driveStraight2(460);
    // driveTurn2(180);
    // driveStraight2(-500);
    // Mogo.set_value(true);


    // driveStraightC(600);
    // driveTurn2(180);
    // driveStraight2(750);
    // LIFT.move(100);








    // Intake.move(127);
    // delay(500);
    // Intake.move(-127);
    // driveStraightC(150);
    // driveTurn2(90);
    // driveClampS(-900, 50, 80);
    // driveTurn2(0);
    // Intake.move(127);
    // driveStraightC(300);
    // driveArcLF(25, 500, 1000);
    // driveStraight2(2250);
    // driveStraightSlow(-700, 70);
    // driveTurn2(-90);
  
    // driveStraightSlow(800, 50);
    // delay(1000);
    // Intake.move(0);
    // LIFT.move(127);
    // delay(500);
    // driveStraightSlow(300, 60);
    // driveStraight2(-500);
    // driveTurn2(180);
    // LIFT.move(0);
    // Intake.move(127);
    // driveStraightC(700);
    // driveStraightSlow(3700, 30);
    // delay(700);
    // driveStraightSlow(500, 40);
    // driveStraightSlow(-500, 50);
    // driveTurn2(-90);
    // driveStraightSlow(600, 50);
    // driveTurn2(30);
    // driveStraightSlow(-450, 50);
    // Mogo.set_value(true);
    }



    if(atn == 7) {
      Mogo.set_value(true);
      LIFT.move(127);
      delay(400);
      driveStraight2(150);
      LIFT.move(-127); //ladybrown down
      delay(200);
      Intake.move(127);
      driveStraightC(-275); 
      driveTurn2(-90);
      driveClampS(-1100, 50, 80); //mogo clamp
      LIFT.move(0);
      driveTurn2(180);
      driveStraight2(800);
    }



    // if(atn == 8)  {

    // }



    // if(atn == 9){
    // }



}

