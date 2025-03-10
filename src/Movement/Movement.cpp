//
// Created by Julius on 16.12.2024.
//
#include "Movement\Movement.h"
#include <Arduino.h>
#include <cmath>
#include <Vector\Vector2.hpp>
#include <IR\IRSensor.hpp>
#include <Algorithm>
#include <elapsedMillis.h>

float multiplier;
elapsedMillis timer_kickoff;

Movement::Movement() : driveVector(0, 0) {} //init the vector to store globally

int Movement::wrapAngle(const int _alpha) {
  int wrapped = (_alpha + 180) % 360;
  if (wrapped < 0) {
    wrapped += 360;
  }
  return wrapped - 180;
}

int Movement::driveToBall(const int _ballDirection, const int _ballDistance, const int _goalDirection, const int _goalDistance)
{
  if (_ballDistance == 0) {
        Serial.println("Error: Division by zero in DriveToBall");
        return 180; // drive straight back to goal
  }

  multiplier = (13 / static_cast<float>(_ballDistance)); // calculate multiplier based on distance
  multiplier = std::max(1.0f, std::min(multiplier, 4.0f)); // cap multiplier between 1 and 4

  if (abs(_ballDirection) <= 100 && abs(_ballDirection) >= 24) { //make sure the robot curves the ball
  	multiplier = std::max(1.75f, std::min(multiplier, 4.0f));
  }

  if (abs(_ballDirection) >= 100)
  {
    multiplier = std::max(1.3f, std::min(multiplier, 4.0f));
  }

  if (_goalDistance <= 24) { // prevent being stuck in goal
    return _ballDirection;
  }

  int _alpha = static_cast<int>(static_cast<float>(_ballDirection) * multiplier); // Calculate the angle to drive to the ball

  if (abs(_ballDirection) <= 24) { // if the ball is in front of the robot drive straight at it
    return _ballDirection;
  }

  if (abs(_alpha) >= 220) { // prevent if the robot is confused because values are to high
    _alpha = std::copysign(220, static_cast<double>(_ballDirection));
  }

  Serial.print(_alpha);
  Serial.print(" : ");
  Serial.print(multiplier);
  Serial.print(" : ");
  Serial.println(_ballDirection);

  return wrapAngle(_alpha);
}

int kickOff() // unused
{
    timer_kickoff = 0;
    while (timer_kickoff < 1500)
    {
      return 0;
    }
    return 0;
}


