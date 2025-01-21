#include "Logic/lop.h"
#include <elapsedMillis.h>
#include "RunningMedian.h"

int previousCompass = 0;
bool lack_of_progress = false;
elapsedMillis timer;
int threshold = 10;

bool logic::lop::check_lop(int _compass) {
    if ((std::abs(_compass) - std::abs(previousCompass)) < threshold) {
        if (timer >= timerDuration) {
            timer = 0; // Reset the timer
            lack_of_progress = true;
        }
    } else {
        timer = 0; // Reset the timer if the compass value changes
        lack_of_progress = false;
    }

    previousCompass = _compass;
    Serial.println(lack_of_progress);
    return lack_of_progress;
}
