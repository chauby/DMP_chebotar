#ifndef _DMP_UTIL_
#define _DMP_UTIL_ 
#include <vector>
#include "DMP.h"
#include "DMPState.h"
#include "TransformSystemDiscrete.h"
#include "CanonicalSystemDiscrete.h"
#include "LWRApproximatorDiscrete.h"
#include "LWRLearningSystem.h"
#include "Trajectory.h"
DMP* createDmpFromArray(double array[][2],int len);
DMP* createDmpFromFile(const char* name);
#endif /* ifndef _DMP_UTIL_ */
