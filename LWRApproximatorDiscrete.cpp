#include "LWRApproximatorDiscrete.h"
#include <stdio.h>

using namespace std;

namespace dmp{

/**
 * @param weights Weights of the Gaussian basis functions
 * @param model_size Number of basis functions to use
 * @param alpha_canonical Parameter alpha of the canonical system
 */
LWRApproximatorDiscrete::LWRApproximatorDiscrete(vector<double> weights,
                int model_size, double alpha_canonical): weights(weights),
                model_size(model_size), alpha_canonical(alpha_canonical)
{
	initBasisFunctions(model_size, alpha_canonical);
}

/**
 * @param model_size Number of basis functions to use
 * @param alpha_canonical Parameter alpha of the canonical system
 */
LWRApproximatorDiscrete::LWRApproximatorDiscrete(int model_size, double alpha_canonical):
	model_size(model_size), alpha_canonical(alpha_canonical)
{
	initBasisFunctions(model_size, alpha_canonical);
}

/**
 * Initializes centers and bandwidths of the Gaussian basis functions
 * @param model_size Number of basis functions
 * @param alpha_canonical Alpha parameter of the canonical system
 */
void LWRApproximatorDiscrete:: initBasisFunctions(int model_size, double alpha_canonical)
{
	centers.resize(model_size);
	bandwidths.resize(model_size);
	int i = 0;

	// Distribute Gaussian centers evenly based on the
	// time development of the canonical system
	for(i = 0; i < model_size; i++){
                centers[i] = exp(-alpha_canonical * i / model_size);
	}

	// Define bandwidths around computed centers
	for(i = 0; i < model_size - 1; i++){
                bandwidths[i] = 1 / ((centers[i+1] - centers[i]) * (centers[i+1] - centers[i]));
	}
	bandwidths[model_size - 1] = bandwidths[model_size - 2];
}

/**
 * Returns the value of the approximated function given DMP state.
 *
 * @param current_state Current DMP state
 * @param start_state Start state of the DMP. Might be needed for scaling of the force
 * @param goal_state Goal state of the DMP. Might be needed for scaling of the force
 */
double LWRApproximatorDiscrete::getValue(DMPState& current_state)
{
    double psi[model_size];
	//TODO bug  psiSum should be init tot 0 before use it;

    double psiSum = 0;
    int i = 0;
    double canonical_state = current_state.getCanonicalState();

    double f = 0.0;

    // Add contribution of each Gaussian to the approximation
    for(i = 0; i < model_size; i++){
        double temp = (canonical_state - centers[i]);
        psi[i] = exp(-(double)bandwidths[i] * temp * temp);
        psiSum += psi[i];        
        f += weights[i] * psi[i];
    }

    // Normalize and multiply by canonical state and amplitude (goal - start)   
	//printf("psiSum:%f canonical_sys:%f",psiSum,canonical_state);
    f = (f / psiSum) * canonical_state;
	//printf("f:%f end",f);
    return f;
}

/**
 * Returns the vector with evaluations of basis functions at the given canonical state
 * @param canonical_state state of the canonical system
 */
vector<double> LWRApproximatorDiscrete::getBasisFunctionVector(double canonical_state)
{
	int i = 0;
	vector<double> result(model_size);

	double psiSum;

	// Compute contribution of each Gaussian
	for(i = 0; i < model_size; i++){
		double temp = (canonical_state - centers[i]);
		result[i] = exp(-(double)bandwidths[i] * temp * temp);
		psiSum += result[i];
	}

	// Normalize
	for (i = 0; i < model_size; i++)
	{
		result[i] = result[i] * canonical_state / psiSum;
	}

	return result;
}

/**
 * Creates an instance copy of the approximator with the given weights
 * @param weights
 * @return
 */
LWRApproximatorDiscrete* LWRApproximatorDiscrete::createInstance(vector<double> weights)
{
    LWRApproximatorDiscrete* approx = new LWRApproximatorDiscrete(weights, model_size, alpha_canonical);
    return approx;
}

LWRApproximatorDiscrete::~LWRApproximatorDiscrete()
{ 
}

}
