/*
 * DataRecorder.h
 *
 *  Record data and save DMP paramters to file
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include "TemporalCoupling.h"
#include "DMPState.h"
#include <vector>

using namespace std;

namespace dmp{
class DataRecorder{

public:
	DataRecorder();
	~DataRecorder();

        void setNumDimensions(int num_dimensionts);
	void addToTrajectory(vector<DMPState> state_vec);
	//void addToTrajectory(DMPState state_vec);
        vector< vector<DMPState> > getCurrentTrajectory();
	void clearTrajectory();

        bool saveWeights(const vector< vector<double> > weights, const char* file_path);
        bool saveTrajectories(const char* file_path);
	//bool saveWeights(vector<double> weights, string file_path);

        vector< vector<double> > loadWeights (const char* file_path);
protected:
        vector< vector<DMPState> > current_trajectory;
        int num_dimensions;

};

}
#endif
