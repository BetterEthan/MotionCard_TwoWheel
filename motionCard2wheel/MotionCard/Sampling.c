/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   Sampling.c
*Author：      Peng Xu
*Date：        2016/10/28
*Description： 采样型值点所用
*		       
*
*Version：     V1.0
*
********************************************************************/

//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
//数组长度存放地址
#define FLASH_NUM  0x08040000 	
//数组信息存放地址
#define FLASH_INFORMATION  0x08040004



#include "Sampling.h"
#include "math.h"
#include "MotionCard.h"
#include "ringBuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "flash.h"
#include "SpeedPlaning.h"

//定时器10ms来一次
_Bool USARTSEND = 0;
/*********************************************************************
* @name 		FunSampling
* @brief  	采取位姿或采取坐标并生成点方向
* @param  	无
* @retval 	无
********************************************************************/
int PoseSampling(void)
{
	static _Bool FIRST_FLAG = 0;
	static Point_t point1={0,0},point2;
	static float anglee,angleeOld;
	static float posAngle = 0.0f;
	KeyPointInf_t pos;
	static float pathOld;
	static int step = 1;
	float tempLenth = 0.0f;
	//对第一点做处理
	if(FIRST_FLAG == 0)
	{
		if(GetPath() > 50)
		{
			//两点的矢量方向
			anglee = CalculateLineAngle(point1,GetPosPresent().point);
			//加入第一点
			pos.point = point1;
			pos.angle = anglee;
			pos.poseAngle = GetAngleZ();
			pos.length = 0;
			//第一点先不对曲率半径赋值
			PutRingBuffer(pos);
			
			angleeOld = anglee;
			pathOld = 0.0f;
			FIRST_FLAG = 1;
		}
	}
	else
	{
		switch(step)
		{
			//采取坐标和角度
			case 1:
					if((GetPath()-pathOld) > 150.0f)
					{
						point2  = GetPosPresent().point;
						posAngle= GetAngleZ();
						pathOld = GetPath();
						step = 2;
					}
					

				break;
			
			//计算第一步所采点曲线的切线方向
			case 2:
					if((GetPath()-pathOld) > 5.0f)
					{
						anglee =  CalculateLineAngle(point2,GetPosPresent().point);
						//求出本段曲线的曲线长度
						tempLenth = CaculateBsplineLen(point1,point2,angleeOld,anglee);
						
						pos.point = point2;
						pos.poseAngle = posAngle;
						pos.angle = anglee;

						//计算总距离长度
						pos.length = tempLenth + GetLength();
						
						//记录该段的曲率半径
						//对直线做处理
						if(fabs(CalculateAngleSub(angleeOld,anglee)) < 0.01f)
						{
							pos.curvatureR = tempLenth/0.0001f;
						}
						else
						{
							pos.curvatureR = fabs(tempLenth/(CalculateAngleSub(angleeOld,anglee)*CHANGE_TO_RADIAN));
						}
						
						SetLength(pos.length);
						PutRingBuffer(pos);
						
						angleeOld = anglee;
						point1 = point2;
						pathOld = GetPath();
						step = 1;
					}
				break;
					
			
			default:
				break;
		}
	}
	return 1;
}


void PoseSamplingLastPoint(void)
{
		Point_t point2,point1;
		KeyPointInf_t pos;
		float tempLenth = 0.0f;
	
		int n = GetCount();
		point1 = GetRingBufferPoint(n);
		point2  = GetPosPresent().point;

		tempLenth = CalculatePoint2PointDistance(point1,point2);
		
		pos.point = point2;
	
		pos.poseAngle = GetAngleZ();
	
		pos.angle = GetRingBufferPointAngle(n);

		//计算总距离长度
		pos.length = tempLenth + GetLength();
		
		//记录该段的曲率半径
		pos.curvatureR = 10000.0f;

		SetLength(pos.length);
		PutRingBuffer(pos);
}




/*********************************************************************
* @name 		CaculateBeginAngle
* @brief  	计算采样开始时第一点和第二点的方向
* @param  	point1 起始点
* @param    point2 结束点
* @retval 	返回起始点和结束点的姿态角度
********************************************************************/
float CaculateBeginAngle(Point_t point1,Point_t point2)
{
	Point_t direction;
	direction.x = point2.x - point1.x;
	direction.y = point2.y - point1.y;
	return CaculateDirectionAngle(direction);
}




/*********************************************************************
* @name 		CaculateProcessAngle
* @brief  	计算采样中自由结束条件结束点的方向
* @param  	point1 起始点
* @param    point2 结束点
* @param    angleLast  起始点的方向
* @retval 	返回矢量的角度 -180~180
********************************************************************/
float CaculateProcessAngle(Point_t point1, Point_t point2, float angleLast)
{
	//常数项
	Point_t P[2];
	//插值点
	//系数矩阵的逆矩阵
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.166666667f, 0.055555555f } };
	//控制到达切矢方向的提前量
	float length = 0.0f;
	//零时控制点坐标
	Point_t dataPoint[2];
	//最终控制点坐标
	Point_t finalDataPoint[6];
	Point_t direction;
	int i, j;


	//所插值点内存开辟																		
	//passPoint = (Point_t*)malloc(sizeof(Point_t) * 30);


	length = sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));


	//对常数项赋值
	P[0].x = 3 * point1.x + length*cos((angleLast) / 180.0f * 3.1415926f);
	P[0].y = 3 * point1.y + length*sin((angleLast) / 180.0f * 3.1415926f);
	P[1].x = 9 * point2.x;
	P[1].y = 9 * point2.y;


	//清零
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}



		for (j = 0; j < 2; j++)
		{
			dataPoint[1].x += invA[1][j] * P[j].x;
			dataPoint[1].y += invA[1][j] * P[j].y;
		}



	finalDataPoint[3] = dataPoint[1];

	finalDataPoint[5] = point2;


	direction.x = -0.5f*finalDataPoint[3].x + 0.5f*finalDataPoint[5].x;
	direction.y = -0.5f*finalDataPoint[3].y + 0.5f*finalDataPoint[5].y;
	
	return CaculateDirectionAngle(direction);

}





/*********************************************************************
* @name 		CaculateDirectionAngle
* @brief  	计算矢量在平面坐标系中的方向角度
* @param  	direction 方向矢量
* @retval 	返回矢量的角度 -180~180
********************************************************************/
float CaculateDirectionAngle(Point_t direction)
{
	Point_t OriginalPoint = {0,0};
 	return CalculateLineAngle(OriginalPoint,direction);
}





//读取flash信息
int ReadFlashPointInformation(void)
{
	u32 numCount;
	
	//读取点个数
	STMFLASH_Read(FLASH_NUM,&numCount,1);

	//读取点信息到ringBuffer
	STMFLASH_Read(FLASH_INFORMATION,GetFristAdress(),numCount*7);
	
	//设置点个数
	SetUpPointer(numCount);
	
	//将最后一点距原点的长度记录为总长度
	SetLength(GetRingBufferPointLen(numCount));
	
	return 1;
}



//采样完成
int PoseSamplingDone(void)
{
	u32 numCount;
	PoseSamplingLastPoint();
	//读取点个数
	numCount = GetCount();
	
	
	//将规划的速度存入ringBuffer
//	SpeedPlaning();
	//将点个数写在flash
	STMFLASH_Write(FLASH_NUM,&numCount,1);
	//将点信息写在flash
	STMFLASH_Write(FLASH_INFORMATION,GetFristAdress(),numCount*7);
	
	return 1;
}












