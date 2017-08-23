/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   roundView.c
*Author：      Peng Xu
*Date：        2016/10/28
*Description： 圆形视野巡迹函数
*		       
*
*Version：     V1.0
*
********************************************************************/



#include "pathFollowing.h"
#include "math.h"
#include "ringBuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "MotionCard.h"
#include "Move.h"


extern int jishuuu;



/*********************************************************************************
* @name 	PathFollowingNew
* @brief	路径跟随函数
* @param	percent 速度的百分比，若为1代表100%所规划的速度运行。范围为大于0,如果超过1，会超过机器人承受速度
* @retval	无
**********************************************************************************/
int PathFollowing(float percent,float vell) 
{
//	static float vell = 200.0f;
	float angle1 = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngle = 0.0f;
	float robotlen = 0.0f;
	static float disRealPos2VirTarget = 0.0f;
	static float disRealPos2VirPos = 0.0f;
	static PointU_t virtualPos,virtualTarget;
	Pose_t presentLine;
	
	//当前点与虚拟位置点距离 和 虚拟位置点到虚拟目标点距离之和
	float VIEW_L = 0.0f;
	
	if(percent < 0 || percent >1)
	{
		return -1;
	}
	
	
	presentLine = GetPosPresent();
	
	//根据采集的几十组数据绘制而成
	VIEW_L = (8.333e-07f * vell * vell * vell + 0.0006667f * vell * vell - 0.2988f * vell + 107.1f) * 0.75f;
	VIEW_L = VIEW_L > 300 ? 500 : VIEW_L;
	VIEW_L = VIEW_L < 100 ? 200 : VIEW_L;
	
	//获取定位系统所计算的机器人实际行走路径长度
	robotlen = GetPath();

	//虚拟位置点
	virtualPos = SerchVirtualPoint(robotlen);
	
	//计算当前点到虚拟位置点的距离
	disRealPos2VirPos = CalculatePoint2PointDistance(presentLine.point,virtualPos.point);
	
	
	if(GetPath() + VIEW_L - disRealPos2VirPos  > robotlen)
	{
		robotlen = GetPath() + VIEW_L - disRealPos2VirPos;
	}
	
	//求取虚拟目标点
	virtualTarget = SerchVirtualPoint2(robotlen);
	
	//计算实际位置距离虚拟目标点距离
	disRealPos2VirTarget = CalculatePoint2PointDistance(presentLine.point,virtualTarget.point);

	float disAdd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;
	if(disAdd > 0)
	{
		AddPath(2*disAdd);
	}
	else
	{
		//如果超出范围，停止更新距离，虚拟目标点不变
		if(disRealPos2VirPos > VIEW_L)
		{
			UpdateLenStop();
		}
		else
		{
			UpdateLenBegin();
		}
	}
	
	//计算当前点到目标点方向角度
	//angle1 = CalculateLineAngle(presentLine.point,virtualTarget.point);
	
	
	//两端点之间角度的插值
//	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(2) , GetRingBufferPointPoseAngle(1));
	
	
	//记录此时刻的目标角度
//	posAngle = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),angleErr*virtualPos.u);
	
//	angularVel = AngleControl(presentLine.direction,posAngle);
	
//	vell = GetRingBufferPointVell(1)+(GetRingBufferPointVell(2) - GetRingBufferPointVell(1))*virtualPos.u;
//	
//	vell = vell*percent;

//	float time;
//	
//	//该段样条曲线首尾的角度差
//	float angErr = GetRingBufferPointPoseAngle(2) - GetRingBufferPointPoseAngle(1);
//	angErr = angErr > 180.0f ? angErr - 360.0f : angErr;
//	angErr = angErr < -180.0f ? 360.0f + angErr : angErr;
//	
//	//粗略计算每两示教点之间的运动的时间
//	time = (GetRingBufferPointLen(2) - GetRingBufferPointLen(1)) / (GetRingBufferPointVell(2) + GetRingBufferPointVell(1)) * 2;
//	
	
//	UsartSendXP3(posAngle,GetAngleZ(),disRealPos2VirPos);
	if(GetPath() < GetLength())
	{
		TwoWheelWalk(virtualTarget.point.x,virtualTarget.point.y,vell);
//		ThreeWheelVelControl(vell,angle1,angularVel + angErr/time*percent);
	}
	else
	{
		TwoWheelWalk(0,0,0);
		//如果距离行走完成 停车
//		ThreeWheelVelControl(disRealPos2VirTarget,angle1,angularVel);
	}
	
	return 1;
}




/*********************************************************************************
* @name 	AngleControl
* @brief	角度闭环控制程序
* @param	anglePresent 当前的角度 单位 度
* @param  angleTarget  目标角度   单位 度
* @retval	无
**********************************************************************************/
float AngleControl(float anglePresent,float angleTarget)
{
	#define P_ANGLE_CONTROL 1.425f
	float angleErr = 0.0f,angularVel = 0.0f;
	//目标角度减去当前角度
	angleErr = CalculateAngleSub(angleTarget,anglePresent);
	
	angularVel = angleErr * P_ANGLE_CONTROL;

	return angularVel;
	
}


















