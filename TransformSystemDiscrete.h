/*
 * TransformSystemDiscrete.h
 *
 *  Spring-damper based transformation system of a DMP for a discrete movement (non-rhythmic)
 *
 *  See A.J. Ijspeert, J. Nakanishi, H. Hoffmann, P. Pastor, and S. Schaal;
 *      Dynamical movement primitives: Learning attractor models for motor behaviors;
 *      Neural Comput. 25, 2 (February 2013), 328-373
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */


#ifndef TRANSFORM_SYSTEM_DISCRETE_H
#define TRANSFORM_SYSTEM_DISCRETE_H

#include "DMPState.h"
#include "TransformationSystem.h"
#include "stdio.h"
namespace dmp{

	class TransformSystemDiscrete : public TransformationSystem{
		public:

			/**
			 * @param alpha Spring parameter
			 *        Best practice: set it to 25
			 * @param beta Damping parameter
			 *        Best practice: set it to alpha / 4
			 *        for a critically damped system
			 */
			TransformSystemDiscrete(double alpha, double beta);

			/**
			 * Returns the next state (e.g. position,velocity,acceleration) of the DMP
			 *
			 * @param current_state Current state of the DMP including current canonical state
			 * @param start_state Start state of the DMP trajectory
			 * @param goal_state Goal state of the DMP trajectory
			 * @param forcing_term Current forcing term, e.g. computed with the FunctionApproximator
			 * @param dt Time difference between previous and current steps
			 * @param tau Time parameter that influences duration of the movement
			 * @param spatial_coupling Value of the coupling the trajectory to an external variable/modality
			 */
			DMPState getNextState(DMPState& current_state, DMPState& start_state, DMPState& goal_state,
					double canonical_state, double forcing_term, double dt, double tau, double spatial_coupling);

			/**
			 * Based on the given DMP state computes the desired/target value of the forcing function
			 * Needed for learning the forcing function based on a given sequence of DMP states (trajectory)
			 *
			 * @param state DMP state
			 * @param start_state Start state of the DMP
			 * @param goal_state Goal state of the DMP
			 * @param tau Time parameter
			 */
			double getDesiredForcingTerm(DMPState& state, DMPState& start_state, DMPState& goal_state, double tau);

			~TransformSystemDiscrete();
		protected:
			double alpha;
			double beta;
			FILE* file_;

		private:

	};
}
#endif
