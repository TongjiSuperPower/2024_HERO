#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

typedef enum{
servo_up,
servo_down,
}servo_control;

extern void servo_task(void const * argument);

#endif
