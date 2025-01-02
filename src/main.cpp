// Import all Lib´s
#include <Arduino.h>
#include "bohlebots.h"
#include "pompeii.h"
#include <PID_v1.h>
#include <Movement\Movement.h>

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
  else if (bot.taster(1, 1)) {
    bot.my_signature = 1;
    Serial.println("Sig 1");
  }
  else if (bot.taster(1, 2)) {
    bot.my_signature = 2;
    Serial.println("Sig 2");
  }
  else if (bot.taster(2, 1)) {
    bot.my_signature = 1;
    Serial.println("Sig 1");
  }
  else if (bot.taster(2, 2)) {
    bot.my_signature = 2;
    Serial.println("Sig 2");
  }
  while (bot.boardtast(4) || bot.boardtast(3) || bot.boardtast(2) || bot.boardtast(1) || bot.taster(1, 1) || bot.taster(1, 2) || bot.taster(2, 1) || bot.taster(2, 2)) {
    bot.warte(5);
  }
}

void updateSensors() {
  if (latest_compass != bot.kompass()) latest_compass = bot.kompass();
  Input = latest_compass;
  adjustRotation.Compute();
  if (abs(Input) <= 4) {
    Output = 0;
  }
  if (bot.ballExists == true) {  // update the ballDirection
    if (latest_ballDirection != bot.ballDirection) {
      latest_ballDirection = bot.ballDirection;
    }
  }
  bot.syncSensors();  // sync all sensors
  goalDirection = bot.goalDirection;
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
    bot.boardled(2, GRUEN);
    controller.play();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 3)  // Debug
  {
    // Set LED
    bot.boardled(2, BLAU);

    // bot.omnidrive(controller.get_x(latest_ballDirection), controller.get_y(latest_ballDirection), -Output, 60);
    // bot.omnidrive(0, 0, -Output, 100);  // test for PID
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

    float _x = controller.get_x(Drive.DriveToBall(latest_ballDirection, bot.ballDistance));
    float _y = controller.get_y(Drive.DriveToBall(latest_ballDirection, bot.ballDistance));
    if (abs(bot.ballDirection) < 15 && bot.ballDistance < 30) {
      _x = 0;
      _y = 0;
    }
    bot.omnidrive(_x, _y, -Output, 60);
     */
    Serial.print(bot.ballDirection);
    Serial.print(" : ");
    Serial.println(goalDirection);
  }
}