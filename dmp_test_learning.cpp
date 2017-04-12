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
DMP* createDmpFromFile(const char* name)
{
	vector<DMPState> trajectory;
	load_trajectory_p(name,trajectory,0.001);
	DMP* dmp = new DMP();
	dmp->dmpInstance(trajectory, 20, 20, 7, 3);
	return dmp;
}

DMP* createDmpFromArray(double array[][2],int len)
{
	vector<DMPState> trajectory;
	convert_trajectory_pt(array,len,trajectory);
	DMP* dmp = new DMP();
	dmp->dmpInstance(trajectory,20, 50,7,3);
	return dmp;
}

void dmpTrainAndReproduce(const char* inputTrajectoryFile,
				const char* inputXFile,
				const char* outputXFile,
				double alpha_trans,
				int model_size,
				double beta_trans_param,
				double alpha_canonical_param,
				double dt)
{
	FILE* foutputX = fopen(outputXFile, "w+");

	vector<DMPState> trajectoryX;
	load_trajectory_pt(inputTrajectoryFile,trajectoryX);
	save_trajectory(inputXFile,trajectoryX);
	DMP dmpX;
	dmpX.dmpInstance(trajectoryX, alpha_trans, model_size, beta_trans_param, alpha_canonical_param);

	DMPState start_stateX(trajectoryX[0].getX(),trajectoryX[0].getXd(),trajectoryX[0].getXdd());
	DMPState goal_stateX(trajectoryX[trajectoryX.size()-1].getX(),
				trajectoryX[trajectoryX.size()-1].getXd(),
				trajectoryX[trajectoryX.size()-1].getXdd());

	dmpX.setStart(start_stateX);
	dmpX.setGoal(goal_stateX);

	vector<DMPState>::iterator iter;
	int i  = 0;
	for(iter = trajectoryX.begin(); iter != trajectoryX.end(); iter++)
	{
		i++;
		DMPState state = dmpX.getNextState(dt,true);
		// cout << i << ": " << state.getX() << " " << state.getXd() << " " << state.getXdd() << endl;
		fprintf(foutputX, "%.7f %.7f %.7f %.7f\n", state.getTime(),state.getX(),state.getXd(),state.getXdd());
	}
	fclose(foutputX);
}

void testX()
{
	const char* inputTrajectoryFile = "swing_leg_spline_trajectory_x.txt";
	const char* inputXFile = "dmpFiles/x_input_trajectory.txt";
	const char* outputXFile = "dmpFiles/x_output_trajectory.txt";
	FILE* foutputX = fopen(outputXFile, "w+");

	vector<DMPState> trajectoryX;
	load_trajectory_pt(inputTrajectoryFile,trajectoryX);
	save_trajectory(inputXFile,trajectoryX);
	DMP dmpX;
	dmpX.dmpInstance(trajectoryX, 10, 20, 7, 3);

	DMPState start_stateX(trajectoryX[0].getX(),trajectoryX[0].getXd(),trajectoryX[0].getXdd());
	DMPState goal_stateX(trajectoryX[trajectoryX.size()-1].getX(),
				trajectoryX[trajectoryX.size()-1].getXd(),
				trajectoryX[trajectoryX.size()-1].getXdd());

	dmpX.setStart(start_stateX);
	dmpX.setGoal(goal_stateX);

	double dt = 0.01003;
	vector<DMPState>::iterator iter;
	int i  = 0;
	for(iter = trajectoryX.begin(); iter != trajectoryX.end(); iter++)
	{
		i++;
		DMPState state = dmpX.getNextState(dt,true);
		// cout << i << ": " << state.getX() << " " << state.getXd() << " " << state.getXdd() << endl;
		fprintf(foutputX, "%.7f %.7f %.7f %.7f\n", state.getTime(),state.getX(),state.getXd(),state.getXdd());
	}
	fclose(foutputX);
}

void testZ()
{
	const char* inputTrajectoryFile = "swing_leg_spline_trajectory_z.txt";
	const char* inputZFile = "dmpFiles/z_input_trajectory.txt";
	const char* outputZFile = "dmpFiles/z_output_trajectory.txt";
	FILE* foutputZ = fopen(outputZFile, "w+");

	vector<DMPState> trajectoryZ;
	load_trajectory_pt(inputZFile,trajectoryZ);
	save_trajectory(inputTrajectoryFile,trajectoryZ);
	DMP dmpX;
	dmpX.dmpInstance(trajectoryZ, 35, 50, 4, 3);

	DMPState start_stateZ(trajectoryZ[0].getX(),trajectoryZ[0].getXd(),trajectoryZ[0].getXdd());
	DMPState goal_stateZ(trajectoryZ[trajectoryZ.size()-1].getX(),
				trajectoryZ[trajectoryZ.size()-1].getXd(),
				trajectoryZ[trajectoryZ.size()-1].getXdd());

	dmpX.setStart(start_stateZ);
	dmpX.setGoal(goal_stateZ);

	double dt = 0.01003;
	vector<DMPState>::iterator iter;
	int i  = 0;
	for(iter = trajectoryZ.begin(); iter != trajectoryZ.end(); iter++)
	{
		i++;
		DMPState state = dmpX.getNextState(dt,true);
		// cout << i << ": " << state.getX() << " " << state.getXd() << " " << state.getXdd() << endl;
		fprintf(foutputZ, "%.7f %.7f %.7f %.7f\n", state.getTime(),state.getX(),state.getXd(),state.getXdd());
	}
	fclose(foutputZ);
}

int main(void)
{
	const char* inputXTrajectoryFile = "swing_leg_spline_trajectory_x.txt";
	const char* inputXFile = "dmpFiles/swing_x_input_trajectory.txt";
	const char* outputXFile = "dmpFiles/swing_x_output_trajectory.txt";
	dmpTrainAndReproduce(inputXTrajectoryFile, inputXFile, outputXFile, 10, 20, 7, 3, 0.01003);

	const char* inputZTrajectoryFile = "swing_leg_spline_trajectory_z.txt";
	const char* inputZFile = "dmpFiles/swing_z_input_trajectory.txt";
	const char* outputZFile = "dmpFiles/swing_z_output_trajectory.txt";
	dmpTrainAndReproduce(inputZTrajectoryFile, inputZFile, outputZFile, 35, 50, 4, 3, 0.01003);

	inputXTrajectoryFile = "support_leg_spline_trajectory_x.txt";
	inputXFile = "dmpFiles/support_x_input_trajectory.txt";
	outputXFile = "dmpFiles/support_x_output_trajectory.txt";
	dmpTrainAndReproduce(inputXTrajectoryFile, inputXFile, outputXFile, 10, 50, 4, 3, 0.01003);

	inputZTrajectoryFile = "support_leg_spline_trajectory_z.txt";
	inputZFile = "dmpFiles/support_z_input_trajectory.txt";
	outputZFile = "dmpFiles/support_z_output_trajectory.txt";
	dmpTrainAndReproduce(inputZTrajectoryFile, inputZFile, outputZFile, 30, 20, 4, 6, 0.01003);

	// testX();
	// testZ();

	return 0;
}
