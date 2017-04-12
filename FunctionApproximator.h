/*
 * FunctionApproximator.h
 *
 *  Abstract class for a non-linear function approximator (forcing function)
 *  that can approximate a given trajectory
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef FUNCTION_APPROXIMATOR_H
#define FUNCTION_APPROXIMATOR_H

#include "DMPState.h"
#include <vector>

namespace dmp{
class FunctionApproximator{
public:

	/**
	 * Returns the value of the approximated function given DMP state.
	 *
	 * @param current_state Current DMP state
	 * @param start_state Start state of the DMP. Might be needed for scaling of the force
	 * @param goal_state Goal state of the DMP. Might be needed for scaling of the force
	 */
        virtual double getValue(DMPState& current_state) = 0;


	virtual ~FunctionApproximator(){};
protected:

private:


};
}
#endif
