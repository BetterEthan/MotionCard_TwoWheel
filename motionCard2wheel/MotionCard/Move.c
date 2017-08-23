#include "Move.h"
#include "math.h"
#include "calculate.h"


#define ALPHA 60.0f 


//三轮控制函数
//speed       单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
void ThreeWheelVelControl(float speed, float direction, float rotationVell)
{

	TriWheelVel_t vell;
	float Vx, Vy;
	float theta;
	float robotR = 0.0f;
	
	robotR = GetRobotRadius();
	rotationVell = rotationVell / CHANGE_TO_ANGLE;
	Vx = speed * cos(direction * CHANGE_TO_RADIAN);
	Vy = speed * sin(direction * CHANGE_TO_RADIAN);

	theta = GetAngleZ();
	
	vell.v1 = (float)(-cos((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx - sin((theta + ALPHA) * CHANGE_TO_RADIAN) * Vy + rotationVell*robotR);
	vell.v2 = (float)(cos(theta * CHANGE_TO_RADIAN) * Vx + sin(theta * CHANGE_TO_RADIAN) * Vy + rotationVell*robotR);
	vell.v3 = (float)(-cos((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx + sin((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell*robotR);
		
	VelControlTriWheel(vell.v1,vell.v2,vell.v3);
}




//返回三个轮子轮速
//speed       单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
//posAngle 机器人的姿态  单位 度
TriWheelVel_t CaculateThreeWheelVel(float speed, float direction, float rotationVell,float angleZ)
{
	TriWheelVel_t vell;
	float Vx, Vy;
	float theta;
	float robotR = 0.0f;
	
	robotR = GetRobotRadius();
	rotationVell = rotationVell / CHANGE_TO_ANGLE;
	Vx = speed * cos(direction * CHANGE_TO_RADIAN);
	Vy = speed * sin(direction * CHANGE_TO_RADIAN);

	theta = angleZ;

	vell.v1 = (float)(-cos((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx - sin((theta + ALPHA) * CHANGE_TO_RADIAN) * Vy + rotationVell*robotR);
	vell.v2 = (float)(cos(theta * CHANGE_TO_RADIAN) * Vx + sin(theta * CHANGE_TO_RADIAN) * Vy + rotationVell*robotR);
	vell.v3 = (float)(-cos((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx + sin((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell*robotR);


	return vell;
}



/*************************************************************************************
* @name        GetTrueVEll
* @param      brief       由三轮的转速计算实际的速度
* @param      wheelVell   三轮的速度
*
*
*************************************************************************************/
TriWheelVel2_t GetTrueVell(TriWheelVel_t wheelVell, float zAngle)
{
	float **B; 
	float **M;
	TriWheelVel2_t trueVell;

	B = CreateMemory(3);
	M = CreateMemory(3);

	float robotR = GetRobotRadius();
	M[0][0] = -cos((60 + zAngle) * CHANGE_TO_RADIAN);
	M[0][1] = -sin((60 + zAngle) * CHANGE_TO_RADIAN);
	M[0][2] = robotR;
	M[1][0] = cos(zAngle * CHANGE_TO_RADIAN);
	M[1][1] = sin(zAngle * CHANGE_TO_RADIAN);
	M[1][2] = robotR;
	M[2][0] = -cos((60 - zAngle) * CHANGE_TO_RADIAN);
	M[2][1] = sin((60 - zAngle) * CHANGE_TO_RADIAN);
	M[2][2] = robotR;

	Gauss(M, B, 3);

	float xVell = B[0][0] * wheelVell.v1 + B[0][1] * wheelVell.v2 + B[0][2] * wheelVell.v3;
	float yVell = B[1][0] * wheelVell.v1 + B[1][1] * wheelVell.v2 + B[1][2] * wheelVell.v3;
	trueVell.rotationVell = B[2][0] * wheelVell.v1 + B[2][1] * wheelVell.v2 + B[2][2] * wheelVell.v3;

	trueVell.rotationVell *= (CHANGE_TO_ANGLE);
	trueVell.speed = sqrt(xVell * xVell + yVell * yVell);
	trueVell.direction = atan2f(yVell, xVell)*CHANGE_TO_ANGLE;
	FreeMemory(B, 3);
	FreeMemory(M, 3);
	return trueVell;
}










