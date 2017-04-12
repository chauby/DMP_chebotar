/*
 * TemporalCoupling.h
 *
 *  Abstract class for a coupling of the temporal development of
 *  the canonical system to some external variable or modality
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef TEMPORAL_COUPLING_H
#define TEMPORAL_COUPLING_H

namespace dmp
{
class TemporalCoupling
{
public:

	/**
	 * Returns coupling value that will be used in the
	 * canonical system to adjust time development of the DMP
	 *
	 * @param state Current DMP state
	 */
	virtual double getValue(double canonical_state) = 0;

	virtual ~TemporalCoupling(){};
protected:

private:

};
}
#endif
