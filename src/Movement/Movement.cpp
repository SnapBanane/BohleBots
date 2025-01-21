//
// Created by Julius on 16.12.2024.
//
#include "Movement\Movement.h"
#include <Arduino.h>
#include <cmath>
#include <Vector\Vector2.hpp>
#include <IR\IRSensor.hpp>
#include <Algorithm>

float multiplier;

Movement::Movement() : driveVector(0, 0) {} //init the vector to store globally

int Movement::WrapAngle(int _alpha) {
  int wrapped = (_alpha + 180) % 360;
  if (wrapped < 0) {
    wrapped += 360;
  }
  return wrapped - 180;
}

int Movement::DriveToBall(const int _ballDirection, const int _ballDistance, const int _goalDirection)
{
  if (_ballDistance == 0) {
        Serial.println("Error: Division by zero in DriveToBall");
        return 180; // drive straight back to goal
  }

  multiplier = (15 / static_cast<float>(_ballDistance)); // calculate multiplier based on distance
  multiplier = std::max(1.0f, std::min(multiplier, 4.0f)); // cap multiplier between 1 and 4

  int _alpha = static_cast<int>(static_cast<float>(_ballDirection) * multiplier); // Calculate the angle to drive to the ball

  /*
  if (abs(_alpha) >= (abs(_ballDirection) + 80)) {
    _alpha = _ballDirection + std::copysign(80, _ballDirection);
  }
  */
  if (abs(_alpha) >= 220) {
    _alpha = std::copysign(220, _ballDirection);
  }
  /*
  if (abs(_ballDirection) <= 60) {
    if ((abs(_ballDirection) * 2) > 100)
    {
      _alpha = std::copysign(100, _ballDirection);
    }
  }
  */
  if (abs(_ballDirection) <= 23) {
    _alpha = _ballDirection / 2;
  }
  /*
  if ((_goalDirection > 0 && _alpha < 0) || (_goalDirection < 0 && _alpha > 0)) { //untested but should work
    _alpha *= -1;
  }
  */
  Serial.print(_alpha);
  Serial.print(" : ");
  Serial.print(multiplier);
  Serial.print(" : ");
  Serial.println(_ballDirection);

  return WrapAngle(_alpha);

}
