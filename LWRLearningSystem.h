/*
 * LWRLearningSystem.h
 *
 * Uses locally-weighted regression (LWR) to learn weights of the non-linear
 * function approximator based on weighted Gaussian basis functions.
 *
 *  See A.J. Ijspeert, J. Nakanishi, H. Hoffmann, P. Pastor, and S. Schaal;
 *      Dynamical movement primitives: Learning attractor models for motor behaviors;
 *      Neural Comput. 25, 2 (February 2013), 328-373
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */ 
#ifndef LWR_LEARNING_SYSTEM_H
#define LWR_LEARNING_SYSTEM_H


#include <vector>
#include "DMPState.h"
#include "LearningSystem.h"
#include "TransformationSystem.h"
#include "CanonicalSystem.h"
#include "FunctionApproximator.h"
#include "LWRApproximatorDiscrete.h"
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace std;

namespace dmp{

class LWRLearningSystem : public LearningSystem{
public:
	/**
	 * @param trans_sys Transformation system of the DMP
	 * @param canonical_sys Canonical system of the DMP
	 */
	LWRLearningSystem (TransformationSystem* trans_sys, CanonicalSystem* canonical_sys);

	/**
	 * Learns and updates weights of the given LWR approximator based on the given trajectory
	 *
	 * @param func_approx LWR approximator to learn
	 * @param trajectory Trajectory to learn
	 * @param tau Time parameter that should be used for learning
	 *        To achieve the same execution speed use the same value
	 *        during the learning and execution
	 * @return Weights of the basis functions
	 */
        vector<double> learnApproximator(LWRApproximatorDiscrete& func_approx, vector<DMPState>& trajectory);

	~LWRLearningSystem();

protected:
	/**
	 * Computes the Moore–Penrose pseudo-inverse of a matrix
	 * @param a Matrix to compute inverse of
	 */
        Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &a);

	TransformationSystem* trans_sys;
	CanonicalSystem* canonical_sys;
private:
	FILE* file_;

};
}
#endif
