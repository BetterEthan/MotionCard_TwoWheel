/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   update.c
*Author：      Peng Xu
*Date：        2016/10/28
*Description： 用于更新陀螺仪数所用
*		       
*
*Version：     
*
********************************************************************/




#include "update.h"
#include "stdint.h"
#include "math.h"

static float g_zAngle		    = 0;   //陀螺仪z方向角度 范围-180~180
static float g_PosX   			= 0;	 //定位系统返回的X坐标
static float g_PosY   			= 0;	 //定位系统返回的Y坐标
static int   g_VelLft 			= 0;   //左轮速度
static int   g_VelRgt 			= 0;	 //右轮速度



/***********************************************************************************
* @name 		SetAngleZ
* @brief  	中断读取设定zangle值
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetAngleZ(float temp)
{
	g_zAngle = temp;
}






/***********************************************************************************
* @name 		GetAngleZ
* @brief  	读取zangle值   人为将角度0旋转到与x轴正方向对齐
* @param  	无
* @retval 	陀螺仪角度
**********************************************************************************/
float GetAngleZ(void)
{
//注意三轮很双轮不同 双轮定位系统中0度为y轴正方向，需要人为加90度处理，三轮定位系统0度为x轴正反向。
	return g_zAngle;
}






/***********************************************************************************
* @name 		SetPosx
* @brief  	中断读取设定X坐标
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetPosx(float temp)
{
	g_PosX = temp;
}






/***********************************************************************************
* @name 		GetPosx
* @brief  	读取X坐标
* @param  	无
* @retval 	X坐标
**********************************************************************************/

float GetPosx(void)
{
	return g_PosX;
}





/***********************************************************************************
* @name 		SetPosy
* @brief  	中断读取设定Y坐标
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetPosy(float temp)
{
	g_PosY = temp;
}





/***********************************************************************************
* @name 		GetPosy
* @brief  	读取Y坐标
* @param  	无
* @retval 	Y坐标
**********************************************************************************/

float GetPosy(void)
{
	return g_PosY;
}





/***********************************************************************************
* @name 		SetVelLft
* @brief  	传递左轮速度
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetVelLft(float temp)
{
	g_VelLft = temp;
}





/***********************************************************************************
* @name 		GetVelLft
* @brief  	读取左轮速度
* @param  	无
* @retval 	g_VelLft
**********************************************************************************/
int GetVelLft(void)
{
	return g_VelLft;
}





/***********************************************************************************
* @name 		SetVelRgt
* @brief  	传递右轮速度
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetVelRgt(float temp)
{
	g_VelRgt = temp;
}





/***********************************************************************************
* @name 		GetVelRgt
* @brief  	读取右轮速度
* @param  	无
* @retval 	g_VelRgt
**********************************************************************************/
int GetVelRgt(void)
{
	return g_VelRgt;
}





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
void CaculatePath(void)
{
	static float posXOld = 0.0f,posYOld = 0.0f;
	float err;
	
	if(CaculateLenFlag == 1)
	{
		err = sqrt((GetPosx() - posXOld)*(GetPosx() - posXOld) + (GetPosy() - posYOld)*(GetPosy() - posYOld));
	
//		USART_OUT(SENDUSART,(uint8_t *)"**********\r\n");
		if(err> 0.5f)
		{
			lengthTwoWheel += err;
		}
	}
	posXOld = GetPosx();
	posYOld = GetPosy();
}

void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}


void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
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
ActLine2 GetPosPresent(void)
{
	ActLine2 pos;
	pos.point.x = g_PosX;
	pos.point.y = g_PosY;
	pos.angle   = g_zAngle;
	return pos;
}

//增加距离
void AddPath(float dis)
{
	lengthTwoWheel += dis;
}


static	ActTrueVell s_trueVell;


//通过定位系统计算所得的实际速度
//需要10ms调用一次
void CcltPoseVell(void)
{
	static ActPoint poseOld;
	static float angleOld;
	s_trueVell.vell = CcltPoint2PointDistance(poseOld,GetPosPresent().point) / 0.01f;
	s_trueVell.angle = CHANGE_TO_ANGLE * atan2f(GetPosy()-poseOld.y,GetPosx()-poseOld.x);
	s_trueVell.angularVell = (GetAngleZ() - angleOld) / 0.01f;
	poseOld = GetPosPresent().point;
	angleOld = GetAngleZ();
}




ActTrueVell GetPoseVell(void)
{
	return s_trueVell;
}








