//
// Created by Jan Schlegel on 08.12.24.
//

#ifndef SARCAT_CM4_25_IRSENSOR_HPP
#define SARCAT_CM4_25_IRSENSOR_HPP

#define IR_ADDRESS 0x55
#include <array>
#include <cmath>
#include <stdexcept>
#include "iostream"
#include "Vector/Vector2.hpp"
//#include <numbers>
#include "Settings.h"

namespace I2C {
    class IRModule {
    public:
        IRModule();

        void update();

        Vector2 getBallVector();

        double abstractToWorldDistance(double distance);
    private:
        int _distance;
        int _direction;
        Vector2 _ballVector;

        void calcBallVector();

        void readModule();

    };
}


#endif //SARCAT_CM4_25_IRSENSOR_HPP
