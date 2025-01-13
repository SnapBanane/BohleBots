//
// made by Julius Gerhardus on 11.01.2025
//
#pragma once
#include "Vector/Vector2.hpp"
namespace I2C {
    class IRModule {
    public:
        IRModule();
        void update();
        Vector2 getBallVector();
        void readModule();
        void calcBallVector();
        double abstractToWorldDistance(double distance);
        double getDirection();
        double getDistance();
        double clip(double value, double min, double max);

    private:
        int _distance;
        int _direction;
        Vector2 _ballVector;
    };
}
// pompeii_irsensor_hpp