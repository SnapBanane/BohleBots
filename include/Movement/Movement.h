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
    int DriveToBall(int _ballDirection, int _ballDistance, int _goalDirection, int _goalDistance); // Calculate an angle to drive to the ball
private:
    I2C::IRModule irModule; // Create an instance of the IRModule
    int WrapAngle(int _alpha); // Wrap the angle between -180 and 180
    Vector2 driveVector; // Vector to store the drive direction
};

#endif //MOVEMENT_H
