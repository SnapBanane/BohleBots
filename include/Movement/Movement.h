//
// Created by Julius on 16.12.2024.
//

#ifndef MOVEMENT_H
#define MOVEMENT_H

class Movement {
public:
    float DriveToBall(int _ballDirection, int _ballDistance);
private:
    float WrapAngle(float _alpha);
};

#endif //MOVEMENT_H
