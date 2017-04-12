/*
 * DMP.h
 *  This implements a general structure DMP that contains
 *  CanonicalSystem, TransformationSystem, FunctionApproximator and external couplings
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef DMP_H
#define DMP_H
#include <vector>
#include <stdlib.h>
#include "DMPState.h"
#include "CanonicalSystem.h"
#include "TransformationSystem.h"
#include "FunctionApproximator.h"
#include "TemporalCoupling.h"
#include "SpatialCoupling.h"
#include "stdio.h"
namespace dmp{

class DMP{

public:

        /**
         */
	DMP();

	/**
	 * @param canonical_sys Canonical system that drives this DMP
	 * @param transform_sys Transformation system used in this DMP
	 * @param func_approx Non-linear function approximator that approximates some trajectory
	 */
	DMP(CanonicalSystem* canonical_sys, TransformationSystem* transform_sys,
		 FunctionApproximator* func_approx);

	/**
	 * @param canonical_sys Canonical system that drives this DMP
	 * @param transform_sys Transformation system used in this DMP
	 * @param func_approx Non-linear function approximator that approximates some trajectory
	 * @param spatial_coupling Provides spatial coupling values of the trajectory
	 */
	DMP(CanonicalSystem* canonical_sys, TransformationSystem* transform_sys,
		 FunctionApproximator* func_approx, SpatialCoupling* spatial_coupling);

	/**
	 * Resets DMP and sets all needed parameters to start execution.
	 * Call this function before beginning any DMP execution.
	 *
	 * @param start_state_new New start state
	 * @param goal_state_new New goal state
	 * @param tau_new Time parameter that influences the length of the execution
	 */
	void start(DMPState& start_state_new, DMPState& goal_state_new, double tau_new);

	/**
	 * Runs one step of the DMP based on the time step dt
	 *
	 * @param dt Time difference between the previous and the current step
	 */
	DMPState getNextState(double dt);

	/**
	 * Runs one step of the DMP based on the time step dt
	 *
	 * @param dt Time difference between the previous and the current step
	 * @param update_canonical_state Set to true if the canonical state should be updated here.
	 *        Otherwise you are responsible for updating it yourself.
	 */
	DMPState getNextState(double dt, bool update_canonical_state);

	/**
	 * Returns current DMP state
	 */
	DMPState getCurrentState();
	void dmpInstance(vector<DMPState>& trajectory, double alpha_trans, int model_size, double beta_trans_param, double alpha_trans_param);
	~DMP();
	void setGoal(DMPState& goal_state_new);
	void setGoal(double x ,double dx,double ddx);

	void setStart(DMPState& start_state_new);
	void setStart(double x ,double dx,double ddx);
	void setTau(double tau_new);
	void setCoupleingTerm(double value);
	//讲状态调整到初始状态
	void resetDmp();
	FILE* file_;

protected:
	CanonicalSystem* canonical_sys;  //正规化系统采用半衰函数将时间映射成为S
	TransformationSystem* transform_sys;//转换系统即是运动与力之间的转化,相当于是反馈系统
	FunctionApproximator* func_approx;//近似函数拟合力的基函数
	SpatialCoupling* spatial_coupling; //偶合因子,DMP 输出的时候能够输出与力叠加作用

	DMPState state;
	DMPState goal_state; 
	DMPState start_state; 

	double tau;
};
}
#endif
