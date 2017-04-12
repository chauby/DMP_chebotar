/*
 * CanonicalSystem.h
 *
 *  Abstract class that contains a transformation system of the DMP.
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef TRANSFORMATION_SYSTEM_H
#define TRANSFORMATION_SYSTEM_H

#include "DMPState.h"

namespace dmp{
class TransformationSystem{
public:

	/**
	 * Returns the next state (e.g. position,velocity,acceleration) of the DMP
	 *
	 * @param current_state Current state of the DMP including current canonical state
	 * @param goal_state Goal state of the DMP trajectory
	 * @param forcing_term Current forcing term, e.g. computed with the FunctionApproximator
	 * @param dt Time difference between previous and current steps
	 * @param tau Time parameter that influences duration of the movement
	 * @param spatial_coupling Value of the coupling the trajectory to an external variable/modality
	 */
        virtual DMPState getNextState(DMPState& current_state, DMPState& start_state, DMPState& goal_state,
                         double forcing_term, double canonical_state, double dt, double tau, double spatial_coupling) = 0;

	/**
	 * Based on the given DMP state computes the desired/target value of the forcing function
	 * Needed for learning the forcing function based on a given sequence of DMP states (trajectory)
	 *
	 * @param state DMP state
	 * @param goal_state Goal state of the DMP
	 * @param tau Time parameter
	 */
        virtual double getDesiredForcingTerm(DMPState& state, DMPState& start_state, DMPState& goal_state, double tau) = 0;

        virtual ~TransformationSystem(){};
protected:

private:

};
}
#endif
