/*
 * LWRDataRecorder.h
 *

 *  Created on: Jan 13, 2014
 *  Author: chebotar
 */


#ifndef LWR_DATA_RECORDER_H
#define LWR_DATA_RECORDER_H

#include "DataRecorder.h"
#include "LWRLearningSystem.h"

namespace dmp{

class LWRDataRecorder : public DataRecorder{

public:
        LWRDataRecorder(LWRLearningSystem* learning_sys, LWRApproximatorDiscrete* lwr_approx);
        bool learnAndSaveWeights(const char* file_path);
//        vector<LWRApproximatorDiscrete> LWRDataRecorder::loadApproximators(const char* file_path);
private:
	LWRLearningSystem* learning_sys;
	LWRApproximatorDiscrete* lwr_approx;
};
}

#endif
