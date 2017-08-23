/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   caculate.h
*Author：      Peng Xu
*Date：        2016/10/21
*Description： 平面的数学计算函数头文件
*
*Version：     V1.0
*
********************************************************************/


#ifndef _CALCULATE_H
#define _CALCULATE_H
#include "MotionCard.h"


//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    0.017453f   
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     57.2958f				
//圆周率
#define PI                  3.1415926f

#define NULL 0



//三轮控制姿态参数
typedef struct
{
	Point_t  point;
	float    angle;
	float    poseAngle;
	float    length;
	float    curvatureR;
	float    vellMax;
}KeyPointInf_t;

typedef struct
{
	float speed;
	float direction;
	float rotationVell;
}TriWheelVel2_t;



//KeyPointInf_t结构体中存放变量所占字节数大小，用于flash存数。
#define BYTESNUM 7

typedef struct
{
	Point_t point;
	float u;
}PointU_t;



float CalculateAngleAdd(float angle1, float angle2);
float CalculateAngleSub(float minuend, float subtrahend);
Point_t CalculateTwoLineIntersection2(Pose_t line1, Pose_t line2);
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd);
Pose_t CalculateLine2(Point_t pointStart, Point_t pointEnd);
float CalculatePoint2PointDistance(Point_t point1, Point_t point2);
float CalculateDisPointToLine2(Point_t point, Pose_t line);
void FreeMemory(float **a, int n);
float **CreateMemory(int n);
void Gauss(float** A, float** B, int n);
void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution);


#endif

