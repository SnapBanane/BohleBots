//
// created by Julius Gerhardus on 12.12.24
//
#include "IR/IRSensor.hpp"
#include <Wire.h>
#include <cmath>
#include <iostream>

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
        double radianDirection = (double) _direction / 64.0 * 2 * PI;

        _ballVector = Vector2(std::sin(radianDirection) * realDistance, std::cos(radianDirection) * realDistance);
    }

    double I2C::IRModule::abstractToWorldDistance(double x) {
        return 663.4261846876087
               - 81.71820769715386 * x
               + 3.805895492702444 * pow(x, 2)
               - 0.07899507898499276 * pow(x, 3)
               + 0.0006191724941177318 * pow(x, 4);
    }
    double I2C::IRModule::getDirection() {
        double _x = _ballVector.getX();
        double _y = _ballVector.getY() * -1;

        return std::atan2(_x, _y) * 180 / PI;
    }