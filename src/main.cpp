// Import all Lib´s
#include "bohlebots.h"
#include "pompeii.h"
#include <PID_v1.h> 

// Init bot
BohleBots bot;          // initiate the bot header
Controller controller(bot);  // initiate the pompeii header
elapsedMillis fahrzeit;

// Ints, you know, change
int modus = 0;
const int MaxAngle_Drive = 5;
int latest_ballDirection;
double Setpoint, Input, Output;            // for PID
double Kp = 0.325, Ki = 0.2, Kd = 0.0276;  // almost perfect || high battery
double BSetpoint, BInput, BOutput;         // second PID
double BKp = 0.4, BKi = 1.6, BKd = 0.150;  // almost perfect
// double Kp = 0.35, Ki = 1.5, Kd = 0.1; 
// double BKp = 0.4, BKi = 1.6, BKd = 0.150;  // save values || low battery
int latest_compass;
int goalDirection;
// test variables below
int angle = 0;
int cycleCounter = 0;
int SetpointMulti = 0;

// Init PID
PID adjustRotation(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID faceBall(&BInput, &BOutput, &BSetpoint, BKp, BKi, BKd, DIRECT);

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
  BSetpoint = 0;  // Ball PID setpoint
  faceBall.SetMode(AUTOMATIC);
  faceBall.SetOutputLimits(-40, 40);
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
  if (bot.boardtast(3)) {
    modus = 2;
    bot.setze_kompass();
  }
  if (bot.boardtast(2)) {
    modus = 3;
    bot.setze_kompass();
  }
  if (bot.taster(1, 1)) {
    bot.my_signature = 1;
    Serial.println("Sig 1");
  }
  if (bot.taster(1, 2)) {
    bot.my_signature = 2;
    Serial.println("Sig 2");
  }
  if (bot.taster(2, 1)) {
    bot.my_signature = 1;
    Serial.println("Sig 1");
  }
  if (bot.taster(2, 2)) {
    bot.my_signature = 2;
    Serial.println("Sig 2");
  }
}

void updateSensors() {
  if (latest_compass != bot.kompass()) latest_compass = bot.kompass();
  adjustRotation.Compute();
  Input = latest_compass + SetpointMulti;
  // Input = goalDirection;
  if (abs(Input) <= 8) {  // 8 degree tolerance
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
    bot.boardled(1, ROT);
    bot.boardled(2, ROT);
    bot.fahre(0, 0, 0);
    bot.motor(4, 100);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 2)  // Main || Play
  {

    bot.boardled(1, GRUEN);
    bot.boardled(2, GRUEN);
    /*
    Input = latest_compass;
    adjustRotation.Compute();
    BInput = latest_ballDirection;
    faceBall.Compute();
    */
    controller.play();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 3)  // Debug
  {
    bot.boardled(1, BLAU);  // Set LED
    bot.boardled(2, BLAU);

    // bot.omnidrive(controller.get_x(latest_ballDirection), controller.get_y(latest_ballDirection), -Output, 60);
    // bot.omnidrive(0, 0, -Output, 100);  // test for PID
    // bot.omnidrive(controller.get_x(bot.ballDirection),controller.get_y(bot.ballDirection),-Output,50);
    /*
    if (bot.goalExists) {
      if (!bot.goalDistance <= 10) {
        bot.omnidrive(controller.get_x(goalDirection), 1, -Output, 100);
        Serial.print("Goal does Exist : ");
        Serial.println(controller.get_x(goalDirection));
      }
    }
    else {
      bot.boardled(1, ROT);
      Serial.println("Goal does not Exist");
      bot.omnidrive(0, 0, -Output, 70);
    }
    */

    //bot.omnidrive(controller.get_x(angle), controller.get_y(angle), -Output, 50);
    //angle += 22.5;
    //if (angle >= 180) angle = -180;

    // Output for serial plotter (no text, just values)
    // Serial.println(latest_compass);
    // Serial.println(goalDirection);
    // Serial.print(" ");
    // Serial.print(Setpoint);
    // Serial.print(" ");
    // Serial.println(latest_ballDirection);
    // Serial.print(bot.hasBall);
    // Serial.print(" : ");
    Serial.println(bot.lightgate);
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
      Kd = input.toFloat();
      adjustRotation.SetTunings(Kp, Ki, Kd);
    }
  }
}