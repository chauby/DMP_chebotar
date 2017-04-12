/*
 * CanonicalSystem.h
 *
 *  Abstract class that contains a canonical system that drives a DMP.
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef CANONICAL_SYSTEM_H
#define CANONICAL_SYSTEM_H

#include "TemporalCoupling.h"

namespace dmp{
class CanonicalSystem{
public:

	/**
	 * Resets canonical system and sets needed parameters.
	 * Call this function before running any DMPs
	 *
	 * @param tau Time parameter that influences speed of the execution
	 */
	virtual double start(double tau) = 0;

	/**
	 * Returns the current canonical state of the system
	 */
	virtual double getCanonicalState() = 0;

	/**
	 * Runs one step of the canonical system and updates the current canonical state
	 * based on the time difference dt from the previous execution
	 *
	 * @param dt Time difference between the current and the previous execution
	 */
	virtual double updateCanonicalState(double dt) = 0;

	virtual ~CanonicalSystem(){};
	virtual void reset() = 0;
protected:
	double canonical_state;
	double tau;
	TemporalCoupling* temporal_coupling;
private:
};

}
#endif
