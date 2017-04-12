/*
 * CanonicalSystemDiscrete.h
 *
 *  Canonical system of a DMP for a discrete movement (non-rhythmic)
 *
 *  See A.J. Ijspeert, J. Nakanishi, H. Hoffmann, P. Pastor, and S. Schaal;
 *      Dynamical movement primitives: Learning attractor models for motor behaviors;
 *      Neural Comput. 25, 2 (February 2013), 328-373
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef CANONICAL_SYSTEM_DISCRETE_H
#define CANONICAL_SYSTEM_DISCRETE_H

#include <stdlib.h>
#include "CanonicalSystem.h"
#include "TemporalCoupling.h"

namespace dmp{

class CanonicalSystemDiscrete : public CanonicalSystem{

public:

	/**
	 * @param alpha Parameter alpha of the canonical system
	 *        Best practice: set it to 1/3 of alpha-parameter of
	 *        the transformation system
	 */
	CanonicalSystemDiscrete(double alpha);

	/**
	 * @param alpha Parameter alpha of the canonical system
	 *        Best practice: set it to 1/3 of alpha-parameter of
	 *        the transformation system
	 * @param temporal_coupling Provides temporal coupling values of the temporal DMP development
	 */
	CanonicalSystemDiscrete(double alpha, TemporalCoupling* temporal_coupling);

	/**
	 * Resets canonical system and sets needed parameters.
	 * Call this function before running any DMPs
	 *
	 * @param tau Time parameter that influences speed of the execution
	 */
	double start(double tau);

	/**
	 * Returns the current canonical state of the system
	 */
	double getCanonicalState();

	/**
	 * Runs one step of the canonical system and updates the current canonical state
	 * based on the time difference dt from the previous execution
	 *
	 * @param dt Time difference between the current and the previous execution
	 */
	double updateCanonicalState(double dt);

	~CanonicalSystemDiscrete();
	void reset();
protected:
	double alpha;
private:

};

}
#endif
