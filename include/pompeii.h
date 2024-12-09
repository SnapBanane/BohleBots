#ifndef POMPEII_H
#define POMPEII_H

#include "bohlebots.h"
#include <PID_v1.h>

class Controller {
private:
  BohleBots& bot;  // Reference to BohleBots object
  int latest_ballDirection;
  int goalDirection;
  double Output;
  int Speed;

public:
  Controller(BohleBots& b)
    : bot(b), latest_ballDirection(0), goalDirection(0), Output(0), Speed(60) {}

  void play() {
    if (!bot.ballExists && !bot.goalExists) {  // if nothing exists, idle
      idle();
    }
    else if (bot.hasBall == 0) {  // drive to ball
      getBall();
      bot.boardled(1, ROT);
    }
    else if (bot.hasBall == 1) {  // attack when has ball
      attack();
      bot.boardled(1, GRUEN);
    }
    else {
      // do nothing and blink led :)
      idle();
    }

  }

  void getBall() {
    int x_vector;
    int y_vector;
    int val = latest_ballDirection / 22.5;
    switch(val)
    {
      case 0: // 0 degrees
        y_vector = 2;
        x_vector = 0;
        break;
      case 1: // ~22.5 degrees
        y_vector = 1;
        x_vector = 1;
        break;
      case 2: // ~45 degrees
        y_vector = 0;
        x_vector = 1;
        break;
      case 3: // ~67.5 degrees
        y_vector = -1;
        x_vector = 1;
        break;
      case 4: // ~90 degrees
        y_vector = -1;
        x_vector = 1;
        break;
      case 5: // ~112.5 degrees
        y_vector = -2;
        x_vector = 1;
        break;
      case 6: // ~135 degrees
        y_vector = -3;
        x_vector = 1;
        break;
      case 7: // ~157.5 degrees
        y_vector = -4;
        x_vector = 1;
        break;
      case -1: // ~-22.5 degrees
        y_vector = 1;
        x_vector = 1;
        break;
      case -2: // ~-45 degrees
        y_vector = 0;
        x_vector = 1;
        break;
      case -3: // ~-67.5 degrees
        y_vector = -1;
        x_vector = 1;
        break;
      case -4: // ~-90 degrees
        y_vector = -1;
        x_vector = 1;
        break;
      case -5: // ~-112.5 degrees
        y_vector = -2;
        x_vector = 1;
        break;
      case -6: // ~-135 degrees
        y_vector = -3;
        x_vector = 1;
        break;
      case -7: // ~-157.5 degrees
        y_vector = -4;
        x_vector = 1;
        break;
      case -8: // -180 degrees
        y_vector = -5;
        x_vector = 0;
        break;
    }

    /*
    if (x_vector != 0) { // if the ball is not infront or behind chase the x direction of the ball
      x_vector = 0
    }
    */
    Serial.print(latest_ballDirection);
    Serial.print(" : ");
    Serial.println(y_vector);

    bot.omnidrive(get_x(latest_ballDirection), y_vector, -Output, Speed); //apply the current vector to the robot
    /*
    if (latest_ballDirection != 0) {  // ball is not in front
      if (latest_ballDirection <= -130 || latest_ballDirection >= 130) {
        bot.boardled(2, ROT);
        bot.omnidrive(get_x(latest_ballDirection), -1, -Output, Speed);
      } 
      
      else {
        bot.boardled(2, BLAU);
        bot.omnidrive(get_x(latest_ballDirection), 1, -Output, Speed);
      }
    } 
    else {
      bot.boardled(2, GRUEN);
      bot.omnidrive(0, 1, -Output, Speed);
    }
    */
    /*
    Serial.print(x_vector);
    Serial.print(" : ");
    Serial.println(y_vector);
    */
  }

  void attack() {
    if (bot.goalExists && bot.hasBall == 1) {
      bot.omnidrive(get_x(goalDirection), 1, -Output, Speed);
    } 
    else idle();
  }

  void idle() {
    bot.omnidrive(0, 0, -Output, Speed);
    bot.boardled(1, AUS);
    bot.warte(40);
    bot.boardled(1, GELB);
  }

  // Add these helper functions
  float get_x(float winkel) {
    float rad = winkel * (PI / 180.0);

    return sin(rad);
  }

  float get_y(float winkel) {
    float rad = winkel * (PI / 180.0);

    return cos(rad);
  }

  void setBallDir(int dir) {
    latest_ballDirection = dir;
  }

  void setGoalDir(int dir) {
    goalDirection = dir;
  }

  void setPIDSpeed(double speed) {
    Output = speed;
  } 

  void Debug() {
    Serial.println("+-------------------+");
    Serial.println("| CONTROLLER DEBUG  |");
    Serial.println("+-------------------+");
    Serial.println("   [ Ball Info ]     ");
    Serial.println("    Direction:  " + String(latest_ballDirection));
    Serial.println("    Exists:     " + String(bot.ballExists ? "Yes" : "No"));
    Serial.println("                 ");
    Serial.println("   [ Bot Info ]      ");
    Serial.println("    Compass:    " + String(bot.kompass()));
    Serial.println("    Has Ball:   " + String(bot.hasBall ? "Yes" : "No"));
    Serial.println("    Light Gate: " + String(bot.lightgate));
    Serial.println("                 ");
    Serial.println("   [ Goal Info ]     ");
    Serial.println("    Direction:  " + String(goalDirection));
    Serial.println("    Exists:     " + String(bot.goalExists ? "Yes" : "No"));
    Serial.println("                 ");
    Serial.println("   [ PID Output ]    ");
    Serial.println("    Output:     " + String(-Output));
    Serial.println("                 ");
    Serial.println("+-------------------+");
    Serial.println("| DEBUG END         |");
    Serial.println("+-------------------+");
  }
};

#endif  // POMPEII_H
