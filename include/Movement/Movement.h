//
// Created by Julius on 16.12.2024.
//
#include "Vector/Vector2.hpp"
#include <IR/IRSensor.hpp>

#ifndef MOVEMENT_H
#define MOVEMENT_H

class Movement {
public:
    Movement(); // Constructor
    int driveToBall(int _ballDirection, int _ballDistance, int _goalDirection, int _goalDistance); // Calculate an angle to drive to the ball
    int kickOff(int _ballDirection, int goalDirection, bool goalExists);
private:
    I2C::IRModule irModule; // Create an instance of the IRModule
    int wrapAngle(int _alpha); // Wrap the angle between -180 and 180
    Vector2 driveVector; // Vector to store the drive direction
};

#endif //MOVEMENT_H
