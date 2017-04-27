#ifndef __ETM_MATH_H
#define __ETM_MATH_H
/*
  This is a collection of 16 bit unsigned math function on PIC30F processors using x16 C
*/

#define ETM_MATH_VERSION 0x0000

unsigned int ETMMath16Add(unsigned int value_1, unsigned int value_2);
/*
  A "safe" 16 bit math function with saturation
  Will return the sum of value_1 and value_2
  If the sum is greater than 0xFFFF, it will return 0xFFFF
*/

unsigned int ETMMath16Sub(unsigned int value_1, unsigned int value_2);
/*
  A "safe" 16 bit math function with saturation
  Will return value_1 minus value_2
  If value_2 is greater than value_1 (which would result in a logical negative number which will be rolled back around to a postive number)
  It will return 0x0000
*/


#endif
