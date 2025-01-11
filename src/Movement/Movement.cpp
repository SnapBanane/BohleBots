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
  if (_ballDistance > 32) {
    multiplier = 1.5;
  } else {
    multiplier = 3.5;
  }
  int _alpha = static_cast<int>(static_cast<float>(_ballDirection) * multiplier);
  if (_alpha > std::abs(_ballDirection + 60) || abs(_alpha) >= 360) {
    if (_ballDirection <= 0) {
      _alpha = _ballDirection - 60;
    } else {
      _alpha = _ballDirection + 60;
    }

  }
  const int _beta = WrapAngle(_alpha);

  Serial.print(_alpha);
  Serial.print(", ");
  Serial.print(_beta);
  Serial.print(" : ");
  Serial.println(_ballDirection);

  return _beta;
  /*
  double circleAroundBallSize = 15;

  double radians = _ballDirection * M_PI / 180.0; // Convert direction to radians
  double x = _ballDistance * std::cos(radians);   // Calculate x component
  double y = _ballDistance * std::sin(radians);   // Calculate y component

  Vector2 driveVector(y, x); // Create a vector from x and y

  irModule.update();
  Vector2 driveVector = irModule.getBallVector();

  bool isBallAligned = std::abs(driveVector.getY()) < 5;  // within 5 cm of the ball
  bool isBehindBall = driveVector.getX() > 0;  // before middle part of bot
  bool isFullyBehindBall = driveVector.getX() > circleAroundBallSize;  // before most forward part of bot

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

    float rotation = std::copysign(M_PI / 2, driveVector.getY());
    driveVector = Vector2::rotate(driveVector, rotation);
    return driveVector;

    return driveVector;
}