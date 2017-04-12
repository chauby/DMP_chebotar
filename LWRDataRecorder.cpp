#include "LWRDataRecorder.h"
#include <stdio.h>
using namespace std;

namespace dmp{

	LWRDataRecorder::LWRDataRecorder(LWRLearningSystem* learning_sys, LWRApproximatorDiscrete* lwr_approx):
		learning_sys(learning_sys), lwr_approx(lwr_approx){

		}

	bool LWRDataRecorder::learnAndSaveWeights(const char* file_path)
	{
		int i,j;
		vector< vector<double> > weights(current_trajectory.size());

		for(i = 0; i < current_trajectory.size(); i++){
			vector<DMPState> traj = current_trajectory[i];

			vector<double> weights_el = learning_sys->learnApproximator(*lwr_approx, traj);
			weights[i] = weights_el;
		}
		return saveWeights(weights, file_path);
	}

	/*
	   vector<LWRApproximatorDiscrete> LWRDataRecorder::loadApproximators(const char* file_path){
	   vector< vector<double> > weights = loadWeights(file_path);

	   vector<LWRApproximatorDiscrete> result(weights.size());

	   for(int i = 0; i < weights.size(); i++){
	   result[i] = lwr_approx->createInstance(weights[i]);
	   }

	   return result;
	   }*/

}
