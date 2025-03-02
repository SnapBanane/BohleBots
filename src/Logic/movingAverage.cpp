#include "Logic/movingAverage.h"

movingAverage::movingAverage() : currentIndex(0), count(0), sum(0) {
  for (int i = 0; i < SIZE; ++i) {
    values[i] = 0;
  }
}

void movingAverage::add(int value) {
  sum -= values[currentIndex];
  values[currentIndex] = value;
  sum += value;
  currentIndex = (currentIndex + 1) % SIZE;
  if (count < SIZE) {
    ++count;
  }
}

double movingAverage::getAverage() const {
  return count == 0 ? 0 : sum / count;
}