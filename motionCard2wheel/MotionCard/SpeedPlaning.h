#ifndef _SPEEDPLANING_H
#define _SPEEDPLANING_H




void CalculateThreeWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree);
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree);
float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell);
void SpeedPlaning(void);
















#endif

