//
// Created by Julius on 16.12.2024.
//

#ifndef MOVEMENT_H
#define MOVEMENT_H

class Movement {
public:
    float DriveToBall(int _ballDirection, int _ballDistance); // Calculate an angle to drive to the ball

    void Score(int _goalDirection, int _ballDirection); // Match the goal angle with the ball angle
private:
    float WrapAngle(float _alpha); // Wrap the angle between -180 and 180
};

#endif //MOVEMENT_H
