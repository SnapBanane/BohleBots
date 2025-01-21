//
// Created by julius on 17.01.2025.
//

#ifndef LOP_H
#define LOP_H
namespace logic {
    class lop {
    public:
      bool check_lop(int _compass);

    private:
      bool lack_of_progress = false;
      int previousCompass = 0;
      const unsigned long timerDuration = 3000; // after 1,5sec lack of progress is true (the rules say the ball gets reset after 3 sec)
    };
}
#endif //LOP_H
