#include "DMP.h"
#include <stdio.h>
#include <vector>
#include <iostream>
#include "TransformSystemDiscrete.h"
#include "CanonicalSystemDiscrete.h"
#include "LWRApproximatorDiscrete.h"
#include "LWRLearningSystem.h"
#include "ImpedanceCoupling.h"
using namespace std;
using namespace dmp;

namespace dmp
{

	/**
	 * @param canonical_sys Canonical system that drives this DMP
	 * @param transform_sys Transformation system used in this DMP
	 * @param func_approx Non-linear function approximator that approximates some trajectory
	 */
	DMP::DMP():canonical_sys(NULL), transform_sys(NULL), func_approx(NULL),
	spatial_coupling(NULL), state(0,0,0), start_state(0,0,0), goal_state(0,0,0),tau(0)
	{

	}

	/**
	 * @param canonical_sys Canonical system that drives this DMP
	 * @param transform_sys Transformation system used in this DMP
	 * @param func_approx Non-linear function approximator that approximates some trajectory
	 */
	DMP::DMP(CanonicalSystem* canonical_sys, TransformationSystem* transform_sys,
			FunctionApproximator* func_approx):

		canonical_sys(canonical_sys), transform_sys(transform_sys), func_approx(func_approx),
		spatial_coupling(NULL),
		state(0,0,0), start_state(0,0,0), goal_state(0,0,0),tau(0)
		{

		}

	/**
	 * @param canonical_sys Canonical system that drives this DMP
	 * @param transform_sys Transformation system used in this DMP
	 * @param func_approx Non-linear function approximator that approximates some trajectory
	 * @param spatial_coupling Provides spatial coupling values of the trajectory
	 */
	DMP::DMP(CanonicalSystem* canonical_sys, TransformationSystem* transform_sys,
			FunctionApproximator* func_approx, SpatialCoupling* spatial_coupling):

		canonical_sys(canonical_sys), transform_sys(transform_sys), func_approx(func_approx),
		spatial_coupling(spatial_coupling),
		state(0,0,0), start_state(0,0,0), goal_state(0,0,0),tau(0)
		{

		}

	/**
	 * Resets DMP and sets all needed parameters to start execution.
	 * Call this function before beginning any DMP execution.
	 *
	 * @param start_state_new New start state
	 * @param goal_state_new New goal state
	 * @param tau_new Time parameter that influences the length of the execution
	 */
	void DMP::start(DMPState& start_state_new, DMPState& goal_state_new, double tau_new)
	{   
		start_state = start_state_new;
		goal_state = goal_state_new;
		state = start_state_new;
		tau = tau_new;
	}

	void DMP::setGoal(DMPState& goal_state_new)
	{   
		goal_state = goal_state_new;
	}

	void DMP::setGoal(double x ,double dx,double ddx)
	{   
		DMPState goal_state_new(x,dx,ddx);
		goal_state = goal_state_new;
	}
	void DMP::setStart(DMPState& start_state_new)
	{   
		start_state = start_state_new;
		state = start_state_new;
	}
	void DMP::setStart(double x ,double dx,double ddx)
	{   
		DMPState start_state_new(x,dx,ddx);
		start_state = start_state_new;
		state = start_state_new;
	}

	void DMP::setTau(double tau_new)
	{   
		tau = tau_new;
	}
	/**
	 * Runs one step of the DMP based on the time step dt
	 *
	 * @param dt Time difference between the previous and the current step
	 */
	DMPState DMP::getNextState(double dt)
	{
		return getNextState(dt, false);
	}

	/**
	 * Runs one step of the DMP based on the time step dt
	 *
	 * @param dt Time difference between the previous and the current step
	 * @param update_canonical_state Set to true if the canonical state should be updated here.
	 *        Otherwise you are responsible for updating it yourself.
	 */
	DMPState DMP::getNextState(double dt, bool update_canonical_state)
	{
		using namespace std;
		double canonical_state = canonical_sys->getCanonicalState();
		state.setCanonicalState(canonical_state);
			
		double spatial_c = 0.0;
		if(spatial_coupling){
			//spatial_c = spatial_coupling->getValue(state); old
			spatial_c = spatial_coupling->getValue(); // kipo
		}

		double forcing_term = func_approx->getValue(state);
		state = transform_sys->getNextState(state, start_state, goal_state,
				forcing_term, canonical_state, dt, tau, spatial_c);
		//这已经是在仿真了
		//printf("forcing_term:%lf spatial_coupling %f,canonical_sys:%f\n",forcing_term,spatial_c,canonical_state);
		if(update_canonical_state)
		{
			canonical_sys->updateCanonicalState(dt);
		}

		return state;
	}

	/**
	 * Returns current DMP state
	 */
	DMPState DMP::getCurrentState()
	{
		return state;
	}

	DMP::~DMP()
	{

	}

	void DMP::dmpInstance(vector<DMPState>& trajectory, double alpha_trans, int model_size, double beta_trans_param, double alpha_trans_param)
	{
		int trajectory_len = trajectory.size();
		// double alpha_trans = 35;
		double beta_trans = alpha_trans / beta_trans_param;
		double alpha_canonical = alpha_trans/ alpha_trans_param ; //这个数字越小越能刻画高带宽的函数
		// file_ = fopen("file","w+");
		//double tau_ = 10; //old
		
		// int  model_size = 40;
		// dmp 中的变量
		canonical_sys = new CanonicalSystemDiscrete(alpha_canonical);
		transform_sys = new TransformSystemDiscrete(alpha_trans, beta_trans);
		func_approx =   new LWRApproximatorDiscrete(model_size, alpha_canonical);
		spatial_coupling = new ImpedanceCoupling();

		LWRApproximatorDiscrete*	func = (LWRApproximatorDiscrete*)func_approx;
		LWRLearningSystem learning_sys(transform_sys, canonical_sys);

		vector<double> weights = learning_sys.learnApproximator(*func, trajectory);
		double tau_ = trajectory[trajectory_len-1].getTime(); //old
		canonical_sys->start(tau_); //这句话的位置十分关键,因为会重启dmp
		tau = tau_ ;
	}

	void DMP::resetDmp()
	{
		state = start_state;
		canonical_sys->reset();
	}

	void DMP::setCoupleingTerm(double value)
	{
		this->spatial_coupling->setValue(value);
	}

}
