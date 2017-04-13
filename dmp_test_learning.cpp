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

void initialDMP(DMP *pDmp,
		vector<DMPState> *pTrajectory,
		const char* inputTrajectoryFile,
		const char* inputXFile,
		const char* outputXFile,
		double alpha_trans,
		int model_size,
		double beta_trans_param,
		double alpha_canonical_param,
		double dt)
{
	if(NULL == pDmp)
	{
		cout << "input argument pDmp can't be NULL" << endl;
		return;
	}

	if(NULL == pTrajectory)
	{
		cout << "input argument pTrajectory can't be NULL" << endl;
		return;
	}

	FILE* foutputX = fopen(outputXFile, "w+");

	load_trajectory_pt(inputTrajectoryFile, *pTrajectory);
	save_trajectory(inputXFile, *pTrajectory);

	vector<DMPState> trajectory = *pTrajectory;

	pDmp->dmpInstance(trajectory, alpha_trans, model_size, beta_trans_param, alpha_canonical_param);

	DMPState start_stateX(trajectory[0].getX(),trajectory[0].getXd(),trajectory[0].getXdd());
	DMPState goal_stateX(trajectory[trajectory.size()-1].getX(),
				trajectory[trajectory.size()-1].getXd(),
				trajectory[trajectory.size()-1].getXdd());

	pDmp->setStart(start_stateX);
	pDmp->setGoal(goal_stateX);

	vector<DMPState>::iterator iter;
	int i  = 0;
	for(iter = trajectory.begin(); iter != trajectory.end(); iter++)
	{
		i++;
		DMPState state = pDmp->getNextState(dt,true);
		// cout << i << ": " << state.getX() << " " << state.getXd() << " " << state.getXdd() << endl;
		fprintf(foutputX, "%.7f %.7f %.7f %.7f\n", state.getTime(),state.getX(),state.getXd(),state.getXdd());
	}
	fclose(foutputX);
}

void loadTrajectory(void)
{

}

void DMPSetStartAndGoalState(DMP dmp, vector<DMPState> trajectory, DMPState startState, DMPState goalState, double dt, const char* outputFile)
{
	FILE* foutputX = fopen(outputFile, "w+");
	dmp.setStart(startState);
	dmp.setGoal(goalState);

	vector<DMPState>::iterator iter;
	int i  = 0;
	for(iter = trajectory.begin(); iter != trajectory.end(); iter++)
	{
		i++;
		DMPState state = dmp.getNextState(dt,true);
		// cout << i << ": " << state.getX() << " " << state.getXd() << " " << state.getXdd() << endl;
		fprintf(foutputX, "%.7f %.7f %.7f %.7f\n", state.getTime(),state.getX(),state.getXd(),state.getXdd());
	}
	fclose(foutputX);
}

int main(void)
{
	DMP dmpSwingX;
	DMP dmpSwingZ;
	DMP dmpSupportX;
	DMP dmpSupportZ;
	vector<DMPState> trajectorySwingX;
	vector<DMPState> trajectorySwingZ;
	vector<DMPState> trajectorySupportX;
	vector<DMPState> trajectorySupportZ;

	/* ===================initial DMP===================== */
	double dt = 0.01003;
	const char* inputXTrajectoryFile = "swing_leg_spline_trajectory_x.txt";
	const char* inputXFile = "dmpFiles/swing_x_input_trajectory.txt";
	const char* outputXFile = "dmpFiles/swing_x_output_trajectory.txt";
	initialDMP(&dmpSwingX, &trajectorySwingX, inputXTrajectoryFile, inputXFile, outputXFile, 10, 20, 7, 3, dt);

	const char* inputZTrajectoryFile = "swing_leg_spline_trajectory_z.txt";
	const char* inputZFile = "dmpFiles/swing_z_input_trajectory.txt";
	const char* outputZFile = "dmpFiles/swing_z_output_trajectory.txt";
	initialDMP(&dmpSwingZ, &trajectorySwingZ, inputZTrajectoryFile, inputZFile, outputZFile, 35, 50, 4, 3, dt);

	inputXTrajectoryFile = "support_leg_spline_trajectory_x.txt";
	inputXFile = "dmpFiles/support_x_input_trajectory.txt";
	outputXFile = "dmpFiles/support_x_output_trajectory.txt";
	initialDMP(&dmpSupportX, &trajectorySupportX, inputXTrajectoryFile, inputXFile, outputXFile, 10, 50, 4, 3, dt);

	inputZTrajectoryFile = "support_leg_spline_trajectory_z.txt";
	inputZFile = "dmpFiles/support_z_input_trajectory.txt";
	outputZFile = "dmpFiles/support_z_output_trajectory.txt";
	initialDMP(&dmpSupportZ, &trajectorySupportZ, inputZTrajectoryFile, inputZFile, outputZFile, 30, 20, 4, 6, dt);

	/* ===================change goal state of swing leg===================== */
	DMPState startStateSwingX(trajectorySwingX[0].getX(), trajectorySwingX[0].getXd(),trajectorySwingX[0].getXdd());
	DMPState goalStateSwingX(trajectorySwingX[trajectorySwingX.size()-1].getX() - 0.3,
				trajectorySwingX[trajectorySwingX.size()-1].getXd(),
				trajectorySwingX[trajectorySwingX.size()-1].getXdd());
	dmpSwingX.resetDmp();
	DMPSetStartAndGoalState(dmpSwingX, trajectorySwingX, startStateSwingX, goalStateSwingX, dt, "dmpFiles/swing_x_trajectory_reproduced.txt");
	usleep(1000);

	DMPState startStateSwingZ(trajectorySwingZ[0].getX(),trajectorySwingZ[0].getXd(),trajectorySwingZ[0].getXdd());
	DMPState goalStateSwingZ(trajectorySwingZ[trajectorySwingZ.size()-1].getX(),
				trajectorySwingZ[trajectorySwingZ.size()-1].getXd(),
				trajectorySwingZ[trajectorySwingZ.size()-1].getXdd());
	dmpSwingZ.resetDmp();
	DMPSetStartAndGoalState(dmpSwingZ, trajectorySwingZ, startStateSwingZ, goalStateSwingZ, dt, "dmpFiles/swing_z_trajectory_reproduced.txt");
	usleep(1000);

	/* ===================change goal state of support leg===================== */
	DMPState startStateSupportX(trajectorySupportX[0].getX(), trajectorySupportX[0].getXd(),trajectorySupportX[0].getXdd());
	DMPState goalStateSupportX(trajectorySupportX[trajectorySupportX.size()-1].getX() + 0.3,
				trajectorySupportX[trajectorySupportX.size()-1].getXd(),
				trajectorySupportX[trajectorySupportX.size()-1].getXdd());
	dmpSupportX.resetDmp();
	DMPSetStartAndGoalState(dmpSupportX, trajectorySupportX, startStateSupportX, goalStateSupportX, dt, "dmpFiles/support_x_trajectory_reproduced.txt");
	usleep(1000);

	DMPState startStateSupportZ(trajectorySupportZ[0].getX(),trajectorySupportZ[0].getXd(),trajectorySupportZ[0].getXdd());
	DMPState goalStateSupportZ(trajectorySupportZ[trajectorySupportZ.size()-1].getX(),
				trajectorySupportZ[trajectorySupportZ.size()-1].getXd(),
				trajectorySupportZ[trajectorySupportZ.size()-1].getXdd());
	dmpSupportZ.resetDmp();
	DMPSetStartAndGoalState(dmpSupportZ, trajectorySupportZ, startStateSupportZ, goalStateSupportZ, dt, "dmpFiles/support_z_trajectory_reproduced.txt");

	return 0;
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

