#include "LWRLearningSystem.h"
#include "stdio.h"
using namespace std;

namespace dmp{

/**
 * @param trans_sys Transformation system of the DMP
 * @param canonical_sys Canonical system of the DMP
 */
LWRLearningSystem::LWRLearningSystem (TransformationSystem* trans_sys, CanonicalSystem* canonical_sys):
			trans_sys(trans_sys), canonical_sys(canonical_sys)
{
	// file_ = fopen("learnfile","w+");
}

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
vector<double> LWRLearningSystem::learnApproximator(LWRApproximatorDiscrete& func_approx,
                vector<DMPState>& trajectory)
{
	int traj_size = trajectory.size();
	int model_size = func_approx.getModelSize();
	double tau = trajectory[traj_size-1].getTime();

	DMPState start_state = trajectory[0];
	DMPState goal_state = trajectory[traj_size - 1];

	Eigen::VectorXd F_target(traj_size); 		// Desired force values
	Eigen::MatrixXd Phi(traj_size, model_size) ;   // Matrix with basis function values

	canonical_sys->start(tau);		    // Reset canonical system
	double old_time = trajectory[0].getTime();  // Set start time to the first trajectory state

	// Iterate over DMP state sequence
	for(int i = 1; i < trajectory.size(); i++)
	{
		DMPState state = trajectory[i];

		// Compute time difference between old state and new state
		double dt = state.getTime() - old_time;
		//printf("dt:%f",dt);
		old_time = state.getTime();

		// Get required canonical state
		double canonical_state = canonical_sys->getCanonicalState();
	             state.setCanonicalState(canonical_state);

		// Compute desired force value for this state
		F_target[i] = trans_sys->getDesiredForcingTerm(state, start_state, goal_state, tau);
		// fprintf(file_, "%lf\n",F_target[i]);
		// Compute basis function contributions at the current canonical state
		vector<double> basis_functions = func_approx.getBasisFunctionVector(canonical_state);
                Phi.row(i) = Eigen::VectorXd::Map(&basis_functions[0], basis_functions.size());
                canonical_sys->updateCanonicalState(dt);
	}

	Eigen::MatrixXd Phi_transpose = Phi.transpose();

	// Compute least-squares solution of the regression with pseudo-inverse
	Eigen::VectorXd weights = pseudoInverse(Phi_transpose * Phi) * Phi_transpose * F_target;

	// Save weights in the function approximator
	vector<double> weights_vec(weights.data(), weights.data() + weights.size());
	func_approx.setWeights(weights_vec);

	return weights_vec;
}

/**
 * Computes the Moore–Penrose pseudo-inverse of a matrix
 * @param a Matrix to compute inverse of
 */
Eigen::MatrixXd LWRLearningSystem::pseudoInverse(const Eigen::MatrixXd &a)
{
	double epsilon = std::numeric_limits<double>::epsilon();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance)
		  .select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

LWRLearningSystem::~LWRLearningSystem()
{
	// fclose(file_);
}

}
