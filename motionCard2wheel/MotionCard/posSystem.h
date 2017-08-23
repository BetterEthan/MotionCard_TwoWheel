#ifndef _POSSYSTEM_H
#define _POSSYSTEM_H
#include "calculate.h"




void CaculatePath(void);

void UpdateLenBegin(void);

void UpdateLenStop(void);

void AddPath(float dis);

int GetPath(void);

Pose_t GetPosPresent(void);

void ClearPathLen(void);




#endif
