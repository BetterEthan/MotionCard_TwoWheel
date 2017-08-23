/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   caculate.c
*Author：      Peng Xu
*Date：        2016/10/21
*Description： 平面的数学计算函数
*
*Version：     V1.0
*
********************************************************************/


#include "math.h"
#include "calculate.h"
#include <stdlib.h>



/*********************************************************************************
* @name 	CalculateAngleAdd
* @brief	对-180,180交界处作处理
* @param	angle1:角度1;
* @param    angle2:角度2;
* @retval   返回相加之后的角度 -180~180
**********************************************************************************/
float CalculateAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}



/*********************************************************************************
* @name 	CalculateAngleSub
* @brief	对-180,180交界处作处理
* @param	minuend: 被减数;
			subtrahend: 减数 A - B,A为被减数，B为减数;
* @retval	返回计算后的角度值 -180~180
*********************************************************************************/
float CalculateAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}

/*********************************************************************************
* @name 	CalculateTwoLineIntersection2
* @brief	计算两条直线的交点
* @param	line1:直线1;
* @param    line2:直线2;
* @retval	返回交点坐标
**********************************************************************************/
Point_t CalculateTwoLineIntersection2(Pose_t line1, Pose_t line2)
{
	Point_t intersection;
	//斜率
	float k1 = 0.0f;
	float k2 = 0.0f;

	//因为浮点运算,未对与x轴垂直的直线处理。
	k1 = tan(line1.direction * CHANGE_TO_RADIAN);
	k2 = tan(line2.direction * CHANGE_TO_RADIAN);

	intersection.x = (line1.point.x*k1 - line1.point.y - line2.point.x * k2 + line2.point.y)
		/ (k1 - k2);
	intersection.y = k1 * (intersection.x - line1.point.x) + line1.point.y;

	return intersection;
}


/*********************************************************************************
* @name 	CalculateLineAngle
* @brief	计算两点矢量方向角度
* @param	pointStart:起始点：
* @param	pointEnd:终止点;
* @retval	两点矢量方向角度 -180~180
*********************************************************************************/
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f范围可以包含-180到180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
* @name 	CalculateLine2
* @brief	两点确定一条直线
* @param	pointStart:起始点;
* @param	pointEnd:终止点;
* @retval   Pose_t形式，包括直线方向角度，直线上一点坐标
*********************************************************************************/
Pose_t CalculateLine2(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;
	Pose_t line2;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	line2.direction = (atan2f(a, b) * CHANGE_TO_ANGLE);
	line2.point = pointStart;
	//atan2f范围可以包含-180到180  
	return line2;
}

/*********************************************************************************
* @name 	CalculatePoint2PointDistance
* @brief	计算点到点的距离
* @param	point1 起始点
* @param	point2 结束点
* @retval   返回两点之间的距离
*********************************************************************************/
float CalculatePoint2PointDistance(Point_t point1, Point_t point2)
{
	float dis;
	dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	return dis;
}





/***********************************************************************************
* @name 	CalculateDisPointToLine2
* @brief	计算点到直线距离 返回值为负代表在直线右侧，为正代表在直线左侧
* @param	point 点坐标
* @param	line  直线参数
* @retval	点到直线的距离
**********************************************************************************/


float CalculateDisPointToLine2(Point_t point, Pose_t line)
{
	float dis, k, b;

	k = tanf(line.direction * CHANGE_TO_RADIAN);
	b = line.point.y - k * line.point.x;
	dis = -(k * point.x + b - point.y) / sqrt(1 + k * k);
	
	return dis;
}


/*********************************************************************************
* @name 	**CreateMemory(int n)
* @brief	开辟n*n的内存，并附初值为0
* @param	n：行数，也是列数
* @retval
**********************************************************************************/
float **CreateMemory(int n)
{
	float **a = NULL;
	int i, j;

	//为二维数组分配row行
	a = (float**)malloc(sizeof(void *)* n);
	//为每列分配n个大小空间
	for (i = 0; i < n; ++i){
		a[i] = (float*)malloc(sizeof(float)* n);
	}
	for (i = 0; i < n; i++){
		for (j = 0; j < n; j++){
			a[i][j] = 0;
		}
	}

	return a;
}

/*********************************************************************************
* @name 	FreeMemory(float **a, int n)
* @brief	释放内存函数
* @param	a：所释放的二位数组首地址；   n：数组行数；
* @retval
**********************************************************************************/
void FreeMemory(float **a, int n)
{
	
	
	int i;
	for (i = 0; i < n; ++i){
		free(a[i]);
	}
	free(a);
}

/*********************************************************************************
* @name 	Gauss
* @brief	采用部分主元的高斯消去法求方阵A的逆矩阵B
* @param	A:输入方阵;B:输出方阵;
* @retval
**********************************************************************************/
void Gauss(float** A, float** B, int n)
{
	int i, j, k;
	float max, temp;
	float **t = NULL;

	t = CreateMemory(n);
	// **t  临时矩阵  
	//将A矩阵存放在临时矩阵t[n][n]中  
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			t[i][j] = A[i][j];
		}
	}
	//初始化B矩阵为单位阵  
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B[i][j] = (i == j) ? (float)1 : 0;
		}
	}
	for (i = 0; i < n; i++)
	{
		//寻找主元  
		max = t[i][i];
		k = i;
		for (j = i + 1; j < n; j++)
		{
			if (fabs(t[j][i]) > fabs(max))
			{
				max = t[j][i];
				k = j;
			}
		}
		//如果主元所在行不是第i行，进行行交换  
		if (k != i)
		{
			for (j = 0; j < n; j++)
			{
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;
				//B伴随交换  
				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}
		//判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵  
		if (t[i][i] == 0)
		{

		}
		//消去A的第i列除去i行以外的各行元素  
		temp = t[i][i];
		for (j = 0; j < n; j++)
		{
			//主对角线上的元素变为1  
			t[i][j] = t[i][j] / temp;
			//伴随计算  			
			B[i][j] = B[i][j] / temp;
		}
		//第0行->第n行  
		for (j = 0; j < n; j++)
		{
			//不是第i行  
			if (j != i)
			{
				temp = t[j][i];
				//第j行元素 - i行元素*j列i行元素  
				for (k = 0; k < n; k++)
				{
					t[j][k] = t[j][k] - t[i][k] * temp;
					B[j][k] = B[j][k] - B[i][k] * temp;
				}
			}
		}
	}


	FreeMemory(t,n);
}


/*************************************************************
* @name Matrix
* @brief 追赶法求解线性方程组
* @constantTerm 线性方程组等号右边的列矩阵
* @solution 解
* @num 行数或列数
* @m 系数矩阵对角数组
* @n 对角上方数组
* @k 对角下方数组
* ***********************************************************/

 void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution)
{
	//b为分解后的下三角矩阵的对角数组
	float* a = NULL;
	a = (float *)malloc(sizeof(float)* num);
	//a为分解后的单位上三角矩阵的对角上方数组
	float* b = NULL;
	b = (float *)malloc(sizeof(float)* (num - 1));
	//c为分解后的单位上三角矩阵的对角上方数组
	float* c = NULL;
	c = (float *)malloc(sizeof(float)* num);
	//x为求解过程中的间接解
	float* x = NULL;
	x = (float *)malloc(sizeof(float)* num);
	int i;

	a[0] = m[0];
	b[0] = n[0] / a[0];

	//给分解后下三角矩阵的对角下方数组c赋值
	for (i = 1; i < num; i++)
	{
		c[i] = k[i];
	}


	//给分解后的单位上三角矩阵的对角上方数组a和分解后的单位上三角矩阵的对角上方数组b赋值
	for (i = 1; i < num - 1; i++)
	{
		a[i] = m[i] - k[i] * b[i - 1];
		b[i] = n[i] / a[i];

	}

	a[num - 1] = m[num - 1] - k[num - 1] * b[num - 2];
	//中间解x的初始值
	x[0] = constantTerm[0] / a[0];

	//给中间解赋值
	for (i = 1; i < num; i++)
	{
		x[i] = (constantTerm[i] - k[i] * x[i - 1]) / a[i];
	}

	//解出最终解
	solution[num - 1] = x[num - 1];

	for (i = num - 1; i > 0; i--)
	{
		solution[i - 1] = x[i - 1] - solution[i] * b[i - 1];
	}
	free(a);
	free(b);
	free(c);
	free(x);
}





