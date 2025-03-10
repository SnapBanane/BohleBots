//
// created by Julius Gerhardus on 12.12.24
//
#include "IR/IRSensor.hpp"
#include <Wire.h>
#include <cmath>
#include <iostream>
#include "Vector/Vector2.hpp"
#include <Arduino.h>

#define irAddress 0x55

I2C::IRModule::IRModule() {
    _distance = 0;
    _direction = 0;
}

void I2C::IRModule::update() {
    readModule();
    calcBallVector();
}

Vector2 I2C::IRModule::getBallVector() {
    return _ballVector;
}

void I2C::IRModule::readModule() {
    Wire.beginTransmission(irAddress);
    Wire.write(0); // Assuming the register to read from is 0
    Wire.endTransmission();

    Wire.requestFrom(irAddress, 2);
    if (Wire.available() == 2) {
        _direction = Wire.read();
        _distance = Wire.read();
    }

    //Serial.println(_direction);
    //Serial.println(_distance);
}

void I2C::IRModule::calcBallVector() {
    double realDistance = abstractToWorldDistance(_distance);
    _direction = clip(_direction, 0, 64);
    double radianDirection = (_direction + 1) / 64.0 * 2 * PI;

    //_ballVector = Vector2(std::sin(radianDirection) * realDistance, std::cos(radianDirection) * realDistance);
    _ballVector = Vector2(-std::cos(radianDirection) * realDistance, std::sin(radianDirection) * realDistance);

    double distanceCorrectionFactor = 1 / (1 + std::sin(clip(std::fabs(PI - radianDirection) - PI / 2, 0, PI)));
    _ballVector *= clip(distanceCorrectionFactor, 0, 1);
   // Serial.print(_ballVector.getX());
   // Serial.print(" : ");
   // Serial.println(_ballVector.getY()

}

double I2C::IRModule::abstractToWorldDistance(double x) {
    return 0.01086308 * std::exp(0.17952033 * x);
}

double I2C::IRModule::getDirection() {
    /*
    double _x = _ballVector.getX();
    double _y = _ballVector.getY() * -1;

    return std::atan2(_x, _y) * 180 / PI;
    */
    _direction = clip(_direction, 0, 64);
    double x = ((_direction+1) * 360.0 / 64.0) - 180;
    x *= -1;
    if (abs(x) > 180) {
        x = std::copysign(180, x);
    }
    // Serial.println(x);
    return x;
}

double I2C::IRModule::getDistance() {
  if ( _distance == 0 ) { return 0; }
  int i = _distance - 33;
  if (i < 1) { i = 1; }
  return i;

/*
    int d = 0;
    d = _distance;
    if (d == 0) { return 0; }
    d -= 40;
    if (d < 0) { d = 1; }
    return d;
 */
}

double I2C::IRModule::clip(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}