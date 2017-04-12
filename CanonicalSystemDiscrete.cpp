#include "CanonicalSystemDiscrete.h"
#include <stdio.h>
namespace dmp{

/**
 * @param alpha Parameter alpha of the canonical system
 *        Best practice: set it to 1/3 of alpha-parameter of
 *        the transformation system
 */
CanonicalSystemDiscrete::CanonicalSystemDiscrete(double alpha): alpha(alpha)
{
	temporal_coupling = NULL;
	canonical_state = 1.0;
}

/**
 * @param alpha Parameter alpha of the canonical system
 *        Best practice: set it to 1/3 of alpha-parameter of
 *        the transformation system
 * @param temporal_coupling Provides temporal coupling values of the temporal DMP development
 */
CanonicalSystemDiscrete::CanonicalSystemDiscrete(double alpha, TemporalCoupling* temp_coupling): alpha(alpha)
{
	temporal_coupling = temp_coupling;
	canonical_state = 1.0;
}

/**
 * Resets canonical system and sets needed parameters.
 * Call this function before running any DMPs
 *
 * @param tau Time parameter that influences speed of the execution
 */
double CanonicalSystemDiscrete::start(double new_tau)
{
	canonical_state = 1.0;
	tau = new_tau;
	return canonical_state;
}

/**
 * Returns the current canonical state of the system
 */
double CanonicalSystemDiscrete::getCanonicalState()
{
	return canonical_state;
}

/**
 * Runs one step of the canonical system and updates the current canonical state
 * based on the time difference dt from the previous execution
 *
 * @param dt Time difference between the current and the previous execution
 */
double CanonicalSystemDiscrete::updateCanonicalState(double dt)
{
	double temporal_c = 0.0;
	if(temporal_coupling)
	{
		temporal_c = temporal_coupling->getValue(canonical_state);
	}
	canonical_state = canonical_state + (-alpha * canonical_state + temporal_c) * dt / tau;
	return canonical_state;
}

CanonicalSystemDiscrete::~CanonicalSystemDiscrete()
{

}
void CanonicalSystemDiscrete::reset()
{
	canonical_state = 1.0;
}


}
