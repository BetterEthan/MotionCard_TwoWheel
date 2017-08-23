#ifndef _SAMPLING_H
#define  _SAMPLING_H



#include  "MotionCard.h"
#include  "calculate.h"


/*********************************************************************
* @name 		CaculateBeginAngle
* @brief  	计算采样开始时第一点和第二点的方向
* @param  	point1 起始点
* @param    point2 结束点
* @retval 	返回起始点和结束点的姿态角度
********************************************************************/
float CaculateBeginAngle(Point_t point1,Point_t point2);


/*********************************************************************
* @name 		CaculateProcessAngle
* @brief  	计算采样中自由结束条件结束点的方向
* @param  	point1 起始点
* @param    point2 结束点
* @param    angleLast  起始点的方向
* @retval 	返回矢量的角度 -180~180
********************************************************************/
float CaculateProcessAngle(Point_t point1, Point_t point2, float angleLast);


/*********************************************************************
* @name 		CaculateDirectionAngle
* @brief  	计算矢量在平面坐标系中的方向角度
* @param  	direction 方向矢量
* @retval 	返回矢量的角度 -180~180
********************************************************************/
float CaculateDirectionAngle(Point_t direction);

void PoseSamplingLastPoint(void);

#endif

