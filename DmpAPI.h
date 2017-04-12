#ifndef DMP_API
#define DMP_API 
#include "DMP.h"
#include "DMPState.h"
#include "TransformSystemDiscrete.h"
#include "CanonicalSystemDiscrete.h"
#include "LWRApproximatorDiscrete.h"
#include "LWRLearningSystem.h"
#include "load_trajectory.h"
DMP* createDmp(const char* name);
#endif /* ifndef DMP_API */
