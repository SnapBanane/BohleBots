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
        return 180; // or handle the error appropriately
  }

  if (abs(_ballDirection) <= 10) {
    multiplier = 0;
  }

  multiplier = (20 / static_cast<float>(_ballDistance)) * 2;
  multiplier = std::max(1.0f, std::min(multiplier, 4.0f));
  int _alpha = static_cast<int>(static_cast<float>(_ballDirection) * multiplier); // Calculate the angle to drive to the ball

  if (abs(_alpha) >= 360 || abs(_alpha) >= (_ballDirection + 80)) { // cap the drive at 180 (exept to near to the ball)
    _alpha = _ballDirection + std::copysign(80, _ballDirection);
  }

  if (abs(_alpha) >= 200) {
    _alpha = std::copysign(200, _ballDirection);
  }

  /*
  Serial.print(_alpha);
  Serial.print(" : ");
  Serial.print(_ballDistance);
  Serial.print(" : ");
  Serial.println(_ballDirection);
  */

  if ((_goalDirection > 0 && _alpha < 0) || (_goalDirection < 0 && _alpha > 0)) { //untested but should work
    _alpha = (360 - abs(_alpha)) % 360;
    if (_alpha > 180) {
      _alpha -= 360;
    }
  }

  return WrapAngle(_alpha);

}

Vector2 Movement::CircleBall(int _ballDistance, int _ballDirection) {
    float circleAroundBallSize = 15;
	  double radians = _ballDirection * M_PI / 180.0; // Convert direction to radians
    double x = _ballDistance * std::cos(radians);   // Calculate x component
    double y = _ballDistance * std::sin(radians);   // Calculate y component

    Vector2 driveVector(y, x); // Create a vector from x and y

    // Calculate the angle to circle around the ball
    float circleAroundBallAngle = std::atan2(driveVector.getY(), driveVector.getX()) + M_PI / 2;

    // Calculate the new position to maintain a distance of 5 units from the ball
    double newX = 5 * std::cos(circleAroundBallAngle);
    double newY = 5 * std::sin(circleAroundBallAngle);

    driveVector.setX(newX);
    driveVector.setY(newY);

    return driveVector;
}