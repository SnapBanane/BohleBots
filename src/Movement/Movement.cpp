//
// Created by Julius on 16.12.2024.
//
#include "Movement\Movement.h"
#include <Arduino.h>
#include <cmath>

float multiplier;

float Movement::WrapAngle(float _alpha) {
  int wrapped = static_cast<int>(_alpha + 180.0) % 360;
  if (wrapped < 0) {
    wrapped += 360;
  }
  return wrapped - 180.0;
}

float Movement::DriveToBall(int _ballDirection, int _ballDistance) {
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

  return _alpha;
}

void Movement::MatchGoalAngle(int _goalDirection, int _ballDirection) {

}