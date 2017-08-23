#include "posSystem.h"
#include "MotionCard.h"
#include "math.h"


 


/***********************************************************************************
* @name 		CaculatePath
* @brief  	计算走过的里程
* @param  	
* @retval 	
**********************************************************************************/
static float lengthTwoWheel = 0.0f;
//0不记录
//1记录距离
//默认初始就开始记录距离
static _Bool CaculateLenFlag = 1;


//通过定位系统计算机器人行走的路径长度
void CaculatePath(void)
{
	static float posXOld = 0.0f,posYOld = 0.0f;
	float err = -0.02f;
	
	if(CaculateLenFlag == 1)
	{
		err = sqrt((GetPosx() - posXOld)*(GetPosx() - posXOld) + (GetPosy() - posYOld)*(GetPosy() - posYOld));
		if(err> 0.5f)
		{
			lengthTwoWheel += err;
		}

	}

	posXOld = GetPosx();
	posYOld = GetPosy();
}


//运行路径长度记录
void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}


//停止路径长度记录
void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
}




//增加距离
void AddPath(float dis)
{
	lengthTwoWheel += dis;
}



/***********************************************************************************
* @name 		GetPath
* @brief  	返回里程数
* @param  	无
* @retval 	lengthTwoWheel
**********************************************************************************/
int GetPath(void)
{
	return lengthTwoWheel;
}



/***********************************************************************************
* @name 		GetPosPresent
* @brief  	返回当前姿态
* @param  	无
* @retval 	
**********************************************************************************/
Pose_t GetPosPresent(void)
{
	Pose_t pos;
	pos.point.x = GetPosx();
	pos.point.y = GetPosy();
	pos.direction   = GetAngleZ();
	return pos;
}



void ClearPathLen(void)
{
	lengthTwoWheel = 0.0f;
	UpdateLenBegin();
}








