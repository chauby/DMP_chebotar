/*
 * SpatialCoupling.h
 *
 *  Abstract class for a coupling of the spatial trajectory of a DMP
 *  to some external variable or modality
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef SPATIAL_COUPLING_H
#define SPATIAL_COUPLING_H

#include "DMPState.h"
#include <vector>

using namespace std;

namespace dmp{
class SpatialCoupling
{
public:

	/**
	 * Returns coupling value that will be used in the
	 * transformation system to adjust motion of the DMP
	 *
	 * @param state Current DMP state
	 */
	virtual double getValue(DMPState& state) = 0;
	virtual double getValue() = 0;
	virtual void setValue(double value) = 0;

	/**
	 * Returns multi-dimensional coupling value that will be used in the
	 * transformation system to adjust motion of the DMP
	 *
	 * @param state Current DMP state
	 */
	//virtual std::vector<double> getValue(std::vector<DMPState>& state) = 0;
	virtual ~SpatialCoupling(){};
protected:

private:
};
}
#endif
