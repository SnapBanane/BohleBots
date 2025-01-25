// Import all Lib´s
#include <Arduino.h>
#include "bohlebots.h"
#include "pompeii.h"
#include <PID_v1.h>
#include <Movement\Movement.h>
#include <Logic/lop.h>

// Init bot
BohleBots bot;          // initiate the bot header
Controller controller(bot);  // initiate the pompeii header
elapsedMillis fahrzeit;
Movement Drive;  // initiate the movement header
logic::lop lop;

// Ints, you know, change
int modus = 0;
const int MaxAngle_Drive = 5;
int latest_ballDirection;
double Setpoint, Input, Output;            // for PID
// double Kp = 0.325, Ki = 0.2, Kd = 0.0276;  // old pid
// double Kp = 0.35, Ki = 1.5, Kd = 0.1; // very old
double Kp = 0.155, Ki = 0.09, Kd = 0.027;
int cycleCounter = 0;
int latest_compass;
int goalDirection;
int flipp_switch = 1;
int SAdd;

// Init PID
PID adjustRotation(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Serial connection
  Serial.begin(115200);
  Serial.println("Starting...");

  bot.set_bot_type(3);  // 3 wheels (omni robot)
  bot.set_pixy(true);   // we have a pixy

  bot.init();  // execute all initiate scripts

  bot.setze_kompass();                     // set default angle physicly
  bot.setCompassDirection(bot.kompass());  // set default angle as a variable (should be 0)

  Setpoint = 0;   // PID setpoint
  adjustRotation.SetMode(AUTOMATIC);
  adjustRotation.SetOutputLimits(-40, 40);
  Serial.println("Done!");

  Serial.println("Finished Setup - Ready to go!");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_buttons() {
  if (bot.boardtast(4)) {
    modus = 1;
  }
  else if (bot.boardtast(3)) {
    modus = 2;
    bot.setze_kompass();
    cycleCounter = 0;
    bot.boardled(2, GELB);
  }
  else if (bot.boardtast(2)) {
    modus = 3;
    bot.setze_kompass();
  }
  else if (bot.boardtast(1)) {
	  if (flipp_switch == 1) {
    	  flipp_switch = 2; // Yellow goal
	      bot.boardled(1, GELB);
      } else {
    	  flipp_switch = 1; // Blue goal
        bot.boardled(1, BLAU);
      }
  }
  while (bot.boardtast(1)) {
    bot.warte(5);
  }
}

void updateSensors() {
  bot.syncSensors();
  bot.my_signature = flipp_switch;
  if(bot.goalExists) goalDirection = -bot.goalDirection;
  latest_compass = bot.kompass();
  Input = SAdd;
  if (std::abs(Input) <= 1) {
    Input = 0;
  }
  adjustRotation.Compute();
  if (bot.ballExists == true) {  // update the ballDirection
    if (latest_ballDirection != bot.ballDirection) {
      latest_ballDirection = bot.ballDirection;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  bot.warte(5);  // default delay || must be here for some reason

  check_buttons();  // check for button updates

  updateSensors();  // update sensor variables

  controller.setBallDir(latest_ballDirection); // pass ball Direction to header

  controller.setGoalDir(goalDirection); // >>

  controller.setPIDSpeed(Output); // >>

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 1)  // Stop moving
  {
    bot.boardled(2, ROT);
    bot.fahre(0, 0, 0);
    bot.motor(4, 0);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 2)  // Main || Play
  {
    bot.motor(4, -50);
    if (cycleCounter >= 30) { bot.boardled(2, AUS); bot.boardled(1, AUS); }
    else { cycleCounter++; }
  	//bot.boardled(2, GELB);
  	int factor = 0;
  	if (lop.check_lop(goalDirection) && !bot.goalExists) {
    	factor = std::copysign(90, goalDirection);
  	} else {
    	lop.check_lop(goalDirection);
  	}

  	if (bot.hasBall == 1) {
        bot.motor(4, -100);
    	bot.boardled(1, GRUEN);
    	SAdd = goalDirection + factor;
    	const float x_drive = controller.get_x(goalDirection) / 4; // Initialize x_drive properly
    	bot.omnidrive(0, 1, -Output, 75); // Use initialized x_drive
  	}
  	else {
    	bot.boardled(1, ROT);
    	const int driveAngle = Drive.DriveToBall(latest_ballDirection, bot.ballDistance, goalDirection, bot.goalDistance);
        Serial.println(driveAngle);
    	bot.omnidrive(controller.get_x(driveAngle), controller.get_y(driveAngle), -Output, 60);
    	SAdd = goalDirection;
  	}
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 3)  // Debug
  {
    // Set LED
    bot.boardled(2, BLAU);

    // bot.omnidrive(controller.get_x(latest_ballDirection), controller.get_y(latest_ballDirection), -Output, 35);
    // if (bot.goalExists==true) bot.omnidrive(0, 0, -Output, 60);  // test for PID
    // bot.omnidrive(controller.get_x(bot.ballDirection),controller.get_y(bot.ballDirection),-Output,50;
    // bot.omnidrive(controller.get_x(angle), controller.get_y(angle), -Output, 50);
    // bot.omnidrive(0, 1, -Output, 20);

    // Output for serial plotter (no text, just values)
    // Serial.println(latest_compass);
    // Serial.println(goalDirection);
    // Serial.print(" ");
    // Serial.print(Setpoint);
    // Serial.print(" ");
    // Serial.println(latest_ballDirection);
    // Serial.print(bot.hasBall);
    // Serial.print(" : ");
    // Serial.println(bot.lightgate);
    // Serial.print(controller.get_x(bot.ballDirection));
    // Serial.print(" : ");
    // Serial.println(controller.get_y(bot.ballDirection));
    // Serial.print(" : ");
    // Serial.println(Setpoint);
    // Serial.print(" : ");
    // Serial.println(cycleCounter);
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      Kp = input.toFloat();
      adjustRotation.SetTunings(Kp, Ki, Kd);
    }
    const int driveAngle = Drive.DriveToBall(latest_ballDirection, bot.ballDistance, goalDirection, bot.goalDistance);
    // bot.omnidrive(controller.get_y(driveAngle), controller.get_x(driveAngle), -Output, 20);
    bot.motor(4, -100);
    Serial.println(bot.lightgate);
  }
}