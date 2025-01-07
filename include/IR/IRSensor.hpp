//
// made by Julius Gerhardus on 12.12.24
//
#include "Vector/Vector2.hpp"
namespace I2C {
    class IRModule {
    public:
        IRModule();
        void update();
        Vector2 getBallVector();
        void readModule();
        void calcBallVector();
        double abstractToWorldDistance(double x);
        double getDirection();
        double clip(double value, double min, double max); // Change return type to double
    private:
        int _distance;
        int _direction;
        Vector2 _ballVector;
    };
}
// pompeii_irsensor_hpp