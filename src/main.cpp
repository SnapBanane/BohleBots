// Import all Lib´s
#include <Arduino.h>
#include "bohlebots.h"
#include "pompeii.h"
#include <PID_v1.h>
#include <Movement\Movement.h>
#include <Vector\Vector2.hpp>

// Init bot
BohleBots bot;          // initiate the bot header
Controller controller(bot);  // initiate the pompeii header
elapsedMillis fahrzeit;
Movement Drive;  // initiate the movement header

// Ints, you know, change
int modus = 0;
const int MaxAngle_Drive = 5;
int latest_ballDirection;
double Setpoint, Input, Output;            // for PID
double Kp = 0.325, Ki = 0.2, Kd = 0.0276;  // almost perfect || high battery
// double Kp = 0.35, Ki = 1.5, Kd = 0.1;
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

  Serial.println("Waiting for Team Button Press...");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_buttons() {
  if (bot.boardtast(4)) {
    modus = 1;
  }
  else if (bot.boardtast(3)) {
    modus = 2;
    bot.setze_kompass();
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
  while (bot.boardtast(4) || bot.boardtast(3) || bot.boardtast(2) || bot.boardtast(1) || bot.taster(1, 1) || bot.taster(1, 2) || bot.taster(2, 1) || bot.taster(2, 2)) {
    bot.warte(5);
  }
}

void updateSensors() {
  bot.syncSensors();
  bot.my_signature = flipp_switch;
  goalDirection = bot.goalDirection * -1;
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
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 2)  // Main || Play
  {
    if (bot.hasBall == 1) {
      bot.boardled(1, GRUEN);
      SAdd = goalDirection / 4;
      bot.omnidrive(0, 1, -Output, 70);
    }
    else {
      bot.boardled(1, ROT);
      const int driveAngle = Drive.DriveToBall(latest_ballDirection, bot.ballDistance, bot.goalDirection);
      bot.omnidrive(controller.get_x(static_cast<float>(driveAngle)), controller.get_y(static_cast<float>(driveAngle)), -Output, 50);
      SAdd = latest_compass;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 3)  // Debug
  {
    // Set LED
    bot.boardled(2, BLAU);

    // bot.omnidrive(controller.get_x(latest_ballDirection), controller.get_y(latest_ballDirection), -Output, 60);
    // if (bot.goalExists==true) bot.omnidrive(0, 0, -Output, 60);  // test for PID
    // bot.omnidrive(controller.get_x(bot.ballDirection),controller.get_y(bot.ballDirection),-Output,50;
    // bot.omnidrive(controller.get_x(angle), controller.get_y(angle), -Output, 50);

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
    /*
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      Kd = input.toFloat();
      adjustRotation.SetTunings(Kp, Ki, Kd);
    }

	*/

  	Serial.println(latest_compass);
    bot.motor(1, 50);
  }
}