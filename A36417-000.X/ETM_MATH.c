#include "ETM_MATH.h"

unsigned int ETMMath16Add(unsigned int value_1, unsigned int value_2) {
  unsigned int sum;
  sum = value_1 + value_2;
  if (sum < value_1) {
    sum = 0xFFFF;
  }
  return sum;
}

unsigned int ETMMath16Sub(unsigned int value_1, unsigned int value_2) {
  if (value_2 > value_1) {
    return 0;
  } else {
    return (value_1 - value_2);
  }
}
