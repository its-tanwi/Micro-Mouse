#include "Stack.h"
#include "Arduino.h"

Stack::Stack(short size) {
  maxSize = size;
  array = new byte[size];
}

bool Stack::push(byte item) {
  if (isFull()) return false;
  array[++top] = item;
  return true;
}

byte* Stack::pop() {
  if (isEmpty()) return nullptr;
  return &array[top--];
}

byte* Stack::peek() {
  if (isEmpty()) return nullptr;
  return &array[top];
}

bool Stack::isEmpty() {
  return top == -1;
}

bool Stack::isFull() {
  return top == maxSize - 1;
}
