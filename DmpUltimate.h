#ifndef _DMP_ULTIMATE_H_
#define _DMP_ULTIMATE_H_ 
#include <vector>
#include "DMP.h"
#include "DMPState.h"
#include "TransformSystemDiscrete.h"
#include "CanonicalSystemDiscrete.h"
#include "LWRApproximatorDiscrete.h"
#include "LWRLearningSystem.h"
#include "Trajectory.h"

/**
 * [createDmp]
 * @param  trajectory              [trajectory to create a DMP, including time and positon]
 * @param  alpha_trans           [description]
 * @param  model_size            [description]
 * @param  beta_trans_param      [description]
 * @param  alpha_canonical_param [description]
 * @param  dt                    [delta time]
 * @return                       [description]
 */
DMP* createDmp(vector<DMPState> trajectory, double alpha_trans, int model_size, double beta_trans_param, double alpha_canonical_param);

/**
 * [loadTrajectory description]
 * @param inputTrajectoryFile [description]
 * @param trajectory          [description]
 */
int loadTrajectory(const char* inputTrajectoryFile, vector<DMPState> *trajectory);

/**
 * [DMPSetStartAndGoalState description]
 * @param dmp        [description]
 * @param startState [description]
 * @param goalState  [description]
 */
void DMPSetStartAndGoalState(DMP *dmp, DMPState startState, DMPState goalState);

#endif
