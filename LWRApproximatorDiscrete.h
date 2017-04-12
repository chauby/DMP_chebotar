/*
 * LWRApproximatorDiscrete.h
 *
 *  Non-linear function approximator based on weighted Gaussian basis functions for discrete movements.
 *  Can be learned with help of locally-weighted regression (LWR)
 *
 *  See A.J. Ijspeert, J. Nakanishi, H. Hoffmann, P. Pastor, and S. Schaal;
 *      Dynamical movement primitives: Learning attractor models for motor behaviors;
 *      Neural Comput. 25, 2 (February 2013), 328-373
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */


#ifndef LWR_APPROXIMATOR_DISCRETE_H
#define LWR_APPROXIMATOR_DISCRETE_H

#include <vector>
#include "math.h"
#include "FunctionApproximator.h"
#include "DMPState.h"

using namespace std;
namespace dmp{

class LWRApproximatorDiscrete : public FunctionApproximator{

public:

	/**
	 * @param weights Weights of the Gaussian basis functions
	 * @param model_size Number of basis functions to use
	 * @param alpha_canonical Parameter alpha of the canonical system
	 */
	LWRApproximatorDiscrete(vector<double> weights, int model_size, double alpha_canonical);

	/**
	 * @param model_size Number of basis functions to use
	 * @param alpha_canonical Parameter alpha of the canonical system
	 */
	LWRApproximatorDiscrete(int model_size, double alpha_canonical);

	/**
	 * Returns the value of the approximated function given DMP state.
	 *
	 * @param current_state Current DMP state	 
	 */
	double getValue(DMPState& current_state);

	/**
	 * Returns the vector with evaluations of basis functions at the given canonical state
	 * @param canonical_state state of the canonical system
	 */
	vector<double> getBasisFunctionVector(double canonical_state);

	/**
	 * Returns the number of basis functions
	 */
	int getModelSize(){ return model_size; }

	/**
	 * Returns basis function weights
	 */
	vector<double> getWeights(){ return weights; }

	/**
	 * @param new_weights New basis function weights
	 */
	void setWeights(vector<double> new_weights){ weights = new_weights; }

	/**
	 * Creates an instance copy of the approximator with the given weights
	 * @param weights
	 * @return
	 */
	LWRApproximatorDiscrete* createInstance(vector<double> weights);

	~LWRApproximatorDiscrete();
protected:
	int model_size;
	double alpha_canonical;
private:
	/**
	 * Initializes centers and bandwidths of the Gaussian basis functions
	 * @param model_size Number of basis functions
	 * @param alpha_canonical Alpha parameter of the canonical system
	 */
	void initBasisFunctions(int model_size, double alpha_canonical);
	vector<double> weights;
	vector<double> centers;
	vector<double> bandwidths;
};
}
#endif
