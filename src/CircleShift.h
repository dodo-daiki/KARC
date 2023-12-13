#ifndef CIRCLESHIFT_h
#define CIRCLESHIFT_h

#include <Arduino.h>

int CircleShift(int AllNum, int StartNum, int Count){
    int result = (StartNum + Count) % AllNum;
    if(result < 0) result += AllNum;
    return result;
}

#endif