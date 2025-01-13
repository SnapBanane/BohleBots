//
// Created by Julius on 16.12.2024.
//
#include "Movement\Movement.h"
#include <Arduino.h>
#include <cmath>
#include <Vector\Vector2.hpp>
#include <IR\IRSensor.hpp>

float multiplier;

Movement::Movement() : driveVector(0, 0) {} //init the vector to store globally

int Movement::WrapAngle(int _alpha) {
  int wrapped = (_alpha + 180) % 360;
  if (wrapped < 0) {
    wrapped += 360;
  }
  return wrapped - 180;
}

int Movement::DriveToBall(const int _ballDirection, const int _ballDistance)
{
  if (_ballDistance == 1) { // Very Close to ball = drive in circle around ball
    return WrapAngle(_ballDirection + std::copysign(90, _ballDirection));
  }

  if (_ballDistance == 3) { // Farther away = drive smaller circle around ball
    multiplier = 2;
  }

  if (_ballDistance > 3) { // far away - infinity = drive straight to ball
    multiplier = 2 / (_ballDistance / 4);
  }

  int _alpha = static_cast<int>(static_cast<float>(_ballDirection) * multiplier); // Calculate the angle to drive to the ball

  if (abs(_alpha) >= 180) { // cap the drive at 180 (exept to near to the ball)
    _alpha = 180;
  }

  Serial.print(_alpha);
  Serial.print(" : ");
  Serial.print(_ballDistance);
  Serial.print(" : ");
  Serial.println(_ballDirection);

  return WrapAngle(_alpha);

  /*
  double circleAroundBallSize = 15;

  double radians = _ballDirection * M_PI / 180.0; // Convert direction to radians
  double x = _ballDistance * std::cos(radians);   // Calculate x component
  double y = _ballDistance * std::sin(radians);   // Calculate y component

  Vector2 driveVector(y, x); // Create a vector from x and y

   bool isBallAligned = std::fabs(driveVector.getY()) < 5; // is centered behind the ball
   bool isBehindBall = driveVector.getX() > 0;  // before middle part of bot
   bool isFullyBehindBall = driveVector.getX() > 5;  // before most forward part of bot

  if (isBallAligned && isBehindBall) {
    driveVector.setY(driveVector.getY() * 2);
    Serial.println("Aligned and behind");
    return driveVector;
  }

  if (driveVector.getMagnitude() <= circleAroundBallSize) {
    Serial.print("Original driveVector: ");
    Serial.print(driveVector.getX());
    Serial.print(", ");
    Serial.println(driveVector.getY());

    const double rotation = std::copysign(M_PI / 2, driveVector.getY());
    driveVector = Vector2::rotate(driveVector, rotation);

    Serial.print("Rotated driveVector: ");
    Serial.print(driveVector.getX());
    Serial.print(", ");
    Serial.println(driveVector.getY());

    Serial.println("magnitude <= circleAroundBallSize");
    return driveVector;
  }

  if (!isFullyBehindBall) {
    double magnitude = driveVector.getMagnitude();
    if (magnitude >= circleAroundBallSize) {
      double circleAroundBallAngle = std::asin(circleAroundBallSize / magnitude);
      driveVector = Vector2::rotate(driveVector, std::copysign(circleAroundBallAngle, driveVector.getY()));
    }
    Serial.println("not fully behind ball");
    return driveVector;
  }

  driveVector.setX(driveVector.getX() - (circleAroundBallSize - 5));
  Serial.println("nix trifft zu");
  return driveVector;
  */
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