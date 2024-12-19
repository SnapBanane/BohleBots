//
// created by Julius Gerhardus on 12.12.24
//
#include "IR/IRSensor.hpp"
#include <Wire.h>

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
    }

    void I2C::IRModule::calcBallVector() {
        double realDistance = abstractToWorldDistance(_distance);
        double radianDirection = (double) _direction / 64.0 * 2 * std::numbers::pi;

        _ballVector = Vector2(std::sin(radianDirection) * realDistance, std::cos(radianDirection) * realDistance);
    }

    double I2C::IRModule::abstractToWorldDistance(double x) {
        return 663.4261846876087
               - 81.71820769715386 * x
               + 3.805895492702444 * pow(x, 2)
               - 0.07899507898499276 * pow(x, 3)
               + 0.0006191724941177318 * pow(x, 4);
    }