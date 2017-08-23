#ifndef _MOVE_H
#define _MOVE_H
#include "MotionCard.h"
#include "calculate.h"






void ThreeWheelVelControl(float speed, float direction, float rotationVell);
TriWheelVel_t CaculateThreeWheelVel(float speed, float direction, float rotationVell,float angleZ);

TriWheelVel2_t GetTrueVell(TriWheelVel_t wheelVell, float zAngle);



 










#endif
