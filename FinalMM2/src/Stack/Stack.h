#ifndef Stack_h
#define Stack_h

#include "Arduino.h"

class Stack {
  public:
    Stack(short size);
    bool push(byte item);
    byte* pop();
    byte* peek();
    bool isEmpty();
    short top = -1;
    short maxSize = 0;
    byte* array;
    bool isFull();
};

#endif
