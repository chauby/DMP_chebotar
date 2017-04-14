#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include "DMP.h"
#include "DMPState.h"
#include "TransformSystemDiscrete.h"
#include "CanonicalSystemDiscrete.h"
#include "LWRApproximatorDiscrete.h"
#include "LWRLearningSystem.h"
#include "Trajectory.h"

using namespace std;
using namespace dmp;

/**
 * [createDmp description]
 * @param  fileName              [description]
 * @param  alpha_trans           [description]
 * @param  model_size            [description]
 * @param  beta_trans_param      [description]
 * @param  alpha_canonical_param [description]
 * @return                       [description]
 */
DMP* createDmp(vector<DMPState> trajectory, double alpha_trans, int model_size, double beta_trans_param, double alpha_canonical_param)
{
	DMP* dmp = new DMP();
	dmp->dmpInstance(trajectory, alpha_trans, model_size, beta_trans_param, alpha_canonical_param);
	return dmp;
}

/**
 * [loadTrajectory description]
 * @param inputTrajectoryFile [description]
 * @param trajectory          [description]
 */
int loadTrajectory(const char* inputTrajectoryFile, vector<DMPState> *trajectory)
{
	return(load_trajectory_pt(inputTrajectoryFile, *trajectory));
}

/**
 * [DMPSetStartAndGoalState description]
 * @param dmp        [description]
 * @param startState [description]
 * @param goalState  [description]
 */
void DMPSetStartAndGoalState(DMP *dmp, DMPState startState, DMPState goalState)
{
	dmp->setStart(startState);
	dmp->setGoal(goalState);
}
