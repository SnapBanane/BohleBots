// Import all Lib´s
#include <Arduino.h>
#include <Logic/lop.h>
#include <Movement\Movement.h>
#include <PID_v1.h>
#include "bohlebots.h"
#include "pompeii.h"

// Init bot
BohleBots bot; // initiate the bot header
Controller controller(bot); // initiate the pompeii header
elapsedMillis fahrzeit;
Movement Drive; // initiate the movement header
logic::lop lop;

// Ints, you know, change
int modus = 0;
const int MaxAngle_Drive = 5;
int latest_ballDirection;
double Setpoint, Input, Output; // for PID
// double Kp = 0.325, Ki = 0.2, Kd = 0.0276;  // old pid
// double Kp = 0.35, Ki = 1.5, Kd = 0.1; // very old
double Kp = 0.155, Ki = 0.09, Kd = 0.027;
double ballSetpoint, ballInput, ballOutput;
double ballKp = 0, ballKi = 0, ballKd = 0;
int cycleCounter = 0;
int latest_compass;
int goalDirection;
int flipp_switch = 1;
int SAdd;

// Init PID
PID adjustRotation(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID ballPID(&ballInput, &ballOutput, &ballSetpoint, ballKp, ballKi, ballKd, DIRECT);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Serial connection
  Serial.begin(115200);
  Serial.println("Starting...");

  bot.set_bot_type(3); // 3 wheels (omni robot)
  bot.set_pixy(true); // we have a pixy

  bot.init(); // execute all initiate scripts

  bot.setze_kompass(); // set default angle physicly
  bot.setCompassDirection(bot.kompass()); // set default angle as a variable (should be 0)

  Setpoint = 0; // PID setpoint
  adjustRotation.SetMode(AUTOMATIC);
  adjustRotation.SetOutputLimits(-40, 40);
  ballSetpoint = 0;
  ballPID.SetMode(AUTOMATIC);
  ballPID.SetOutputLimits(0, 1);
  Serial.println("Done!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_buttons()
{
  if (bot.boardtast(4))
  {
    modus = 1;
  }
  else if (bot.boardtast(3))
  {
    // set compass
    bot.setze_kompass();
    // set light cycle counter and toggle led
    cycleCounter = 0;
    bot.boardled(2, GELB);
    // reset PID
    for (int i = 0; i < 20; i++) // make pid think its staying at 0
    {
      Input = 0;
      adjustRotation.Compute();
    }
    // change to playing mode
    modus = 2;
  }
  else if (bot.boardtast(2))
  {
    modus = 3;
    bot.setze_kompass();
  }
  else if (bot.boardtast(1))
  {
    if (modus != 2)
    {
      if (flipp_switch == 1)
      {
        flipp_switch = 2; // Yellow goal
        bot.boardled(1, GELB);
      }
      else
      {
        flipp_switch = 1; // Blue goal
        bot.boardled(1, BLAU);
      }
    }
    while (bot.boardtast(1))
    {
      bot.warte(5);
    }
  }
}

void updateSensors()
{
  // update signature
  bot.my_signature = flipp_switch;
  latest_compass = bot.kompass();
  if (bot.goalExists) { goalDirection = bot.goalDirection; }
  // input for pid
  Input = SAdd;
  if (std::abs(Input) <= 1)
  {
    Input = 0;
  }
  // update PID
  adjustRotation.Compute();
  // update ballDirection
  if (bot.ballExists)
  {
    latest_ballDirection = bot.ballDirection;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  bot.warte(5); // default delay for anti windup

  check_buttons(); // check for button updates

  updateSensors(); // update sensor variables

  controller.setBallDir(latest_ballDirection); // pass ball Direction to header

  controller.setGoalDir(goalDirection); // >>

  controller.setPIDSpeed(Output); // >>

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 1) // Stop moving
  {
    bot.boardled(2, ROT);
    bot.fahre(0, 0, 0);
    bot.motor(4, 0);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 2) // Main || Play
  {
    // choose ANGLE input
    // if (bot.goalExists) ANGLE = static_cast<float>(goalDirection) / 5;
    // else if (bot.goalExists2) ANGLE = bot.goalDirection2 / 5;
    // else ANGLE = static_cast<float>(latest_compass) / 180 * 25;
    float ANGLE = -Output;
    // set dribbler
    bot.motor(4, -100);
    // turn of leds
    if (cycleCounter >= 30)
    {
      bot.boardled(2, AUS);
      bot.boardled(1, AUS);
    }
    else
    {
      cycleCounter++;
    }
    // lack of progress factor
    float factor = 0;
    float factor2 = goalDirection / 5;
    if (lop.check_lop(latest_compass) && !bot.goalExists)
    {
      factor = -1.5;
      // if (!bot.goalExists) { factor2 = -std::copysign(60, bot.goalDirection2); }
      factor2 = std::copysign(40, goalDirection);
    }
    else
    {
      lop.check_lop(latest_compass);
    }
    // if hasBall drive to goal
    if (bot.hasBall == 1)
    {
      //bot.boardled(1, GRUEN);
      SAdd = goalDirection / 5;
      bot.omnidrive(0, 1 + factor, factor2, 75);
    }
    // if not has ball try to get ball
    else
    {
      //bot.boardled(1, ROT);
      const auto driveAngle = static_cast<float>(Drive.driveToBall(latest_ballDirection, bot.ballDistance, goalDirection, bot.goalDistance));
      bot.omnidrive(controller.get_x(driveAngle), controller.get_y(driveAngle), ANGLE, 50);
      SAdd = latest_compass;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (modus == 3) // Debug
  {
    // Set LED
    bot.boardled(2, BLAU);

    // bot.omnidrive(controller.get_x(latest_ballDirection), controller.get_y(latest_ballDirection), -Output, 35);
    // if (bot.goalExists==true) bot.omnidrive(0, 0, -Output, 60);  // test for PID
    // bot.omnidrive(controller.get_x(bot.ballDirection),controller.get_y(bot.ballDirection),-Output,50;
    // bot.omnidrive(controller.get_x(angle), controller.get_y(angle), -Output, 50);
    // bot.omnidrive(0, 0, static_cast<float>(latest_compass) / 180 * 25, 55);

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
    if (Serial.available() > 0)
    {
      String input = Serial.readStringUntil('\n');
      input.trim();
      Kp = input.toFloat();
      adjustRotation.SetTunings(Kp, Ki, Kd);
    }
    // const int driveAngle = Drive.driveToBall(latest_ballDirection, bot.ballDistance, goalDirection,
    // bot.goalDistance); bot.omnidrive(controller.get_y(driveAngle), controller.get_x(driveAngle), -Output, 20);
    bot.motor(4, -100);
    //Serial.println(bot.lightgate);
    /*
      Serial.print(bot.goalExists2);
      Serial.print(" : ");
      Serial.print(bot.goalDirection2);
      Serial.print(" : ");
      Serial.print(bot.goalExists);
      Serial.print(" : ");
      Serial.println(bot.goalDirection);
    */
  }
}
