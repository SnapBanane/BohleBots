#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

class movingAverage {
public:
  movingAverage();
  void add(int value);
  double getAverage() const;

private:
  static const int SIZE = 5;
  int values[SIZE];
  int currentIndex;
  int count;
  double sum;
};

#endif // MOVING_AVERAGE_H