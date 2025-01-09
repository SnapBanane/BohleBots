//
// Created by Julius on 16.12.2024.
//
#include "Movement\Movement.h"
#include <Arduino.h>
#include <cmath>
#include <Vector\Vector2.hpp>

float multiplier;

Movement::Movement() : driveVector(0, 0) {} //init the vector to store globally

float Movement::WrapAngle(float _alpha) {
  int wrapped = static_cast<int>(_alpha + 180.0) % 360;
  if (wrapped < 0) {
    wrapped += 360;
  }
  return wrapped - 180.0;
}

Vector2 Movement::DriveToBall(int _ballDirection, int _ballDistance) {
  /*
  if (_ballDistance > 100) {
    multiplier = 1.5;
  } else {
    multiplier = 3.5;
  }
  int _alpha = (float)_ballDirection * multiplier;
  if (_alpha > std::abs(_ballDirection + 60) || abs(_alpha) >= 360) {
    if (_ballDirection <= 0) {
      _alpha = _ballDirection - 60;
    } else {
      _alpha = _ballDirection + 60;
    }

  }
  int _beta = WrapAngle(_alpha);

  Serial.print(_alpha);
  Serial.print(", ");
  Serial.print(_beta);
  Serial.print(" : ");
  Serial.println(_ballDirection);

  return _beta;
  */
  double radians = _ballDirection * M_PI / 180.0; // Convert direction to radians
  double x = _ballDistance * std::cos(radians);   // Calculate x component
  double y = _ballDistance * std::sin(radians);   // Calculate y component

  Vector2 driveVector(y, x); // Create a vector from x and y

   bool isBallAligned = std::fabs(driveVector.getY()) < 5; // is centered behind the ball
   bool isBehindBall = driveVector.getX() > 0;  // before middle part of bot
   bool isFullyBehindBall = driveVector.getX() > 5;  // before most forward part of bot

   if (isBallAligned && isBehindBall) {
       driveVector.setY(driveVector.getY() * 2);
       return driveVector;
   }

   if (driveVector.getMagnitude() <= 15) {
       float rotation = std::copysign(M_PI / 2, driveVector.getY());
       driveVector = Vector2::rotate(driveVector, rotation);
       return driveVector;
   }

   if (!isFullyBehindBall) {
       float magnitude = driveVector.getMagnitude();
       float circleAroundBallAngle = std::asin(15 / magnitude);
       driveVector = Vector2::rotate(driveVector, std::copysign(circleAroundBallAngle, driveVector.getY()));
	   return driveVector;
   }

   driveVector.setX(driveVector.getX() - (15 - 5));
   return driveVector;
}

Vector2 Movement::CircleBall(int _ballDistance, int _ballDirection) {
	double radians = _ballDirection * M_PI / 180.0; // Convert direction to radians
    double x = _ballDirection * std::cos(radians);   // Calculate x component
    double y = _ballDirection * std::sin(radians);   // Calculate y component

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