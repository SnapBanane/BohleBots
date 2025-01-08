//
// created by Julius Gerhardus on 12.12.24
//
#include "IR/IRSensor.hpp"
#include <Wire.h>
#include <cmath>
#include <iostream>
#include "Vector/Vector2.hpp"

#define irAddress 0x55

const double PI = 3.141592653589793;

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
}

void I2C::IRModule::calcBallVector() {
    double realDistance = abstractToWorldDistance(_distance);
    double radianDirection = (_direction + 1) / 64.0 * 2 * PI;

    _ballVector = Vector2(std::sin(radianDirection) * realDistance, std::cos(radianDirection) * realDistance);

    // double distanceCorrectionFactor = 1 / (1 + std::sin(clip(std::fabs(PI - radianDirection) - PI / 2, 0, PI)));
    // _ballVector *= clip(distanceCorrectionFactor, 0, 1);

    // Uncomment for debugging
    // std::cout << "distance: " << _distance << ", realDistance: " << realDistance << ", vectorDistance: " << _ballVector.rho() << ", direction: " << _direction << std::endl;
    // std::cout << "ballVector: x: " << _ballVector.getX() << ", y: " << _ballVector.getY() << ", direction: " << _direction << std::endl;
}

double I2C::IRModule::abstractToWorldDistance(double x) {
    return 0.01086308 * std::exp(0.17952033 * x);
}

double I2C::IRModule::getDirection() {
    double _x = _ballVector.getX();
    double _y = _ballVector.getY() * -1;

    return std::atan2(_x, _y) * 180 / PI;
}

double I2C::IRModule::clip(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}