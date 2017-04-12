#include "TransformSystemDiscrete.h"
#include "stdio.h"
namespace dmp
{

/**
 * @param alpha Spring parameter
 *        Best practice: set it to 25
 * @param beta Damping parameter
 *        Best practice: set it to alpha / 4
 *        for a critically damped system
 */
TransformSystemDiscrete::TransformSystemDiscrete(double alpha, double beta) : alpha(alpha), beta(beta)
{
	file_ = fopen("fileT","w+");
}

/**
 * Returns the next state (e.g. position,velocity,acceleration) of the DMP
 *
 * @param current_state Current state of the DMP including current canonical state
 * @param goal_state Goal state of the DMP trajectory
 * @param forcing_term Current forcing term, e.g. computed with the FunctionApproximator
 * @param canonical_state Current state of the canonical system
 * @param dt Time difference between previous and current steps
 * @param tau Time parameter that influences duration of the movement
 * @param spatial_coupling Value of the coupling the trajectory to an external variable/modality
 */
DMPState TransformSystemDiscrete::getNextState(DMPState& current_state, DMPState& start_state, DMPState& goal_state,
                double forcing_term, double canonical_state, double dt, double tau, double spatial_coupling)
{
    DMPState next_state = current_state;
    double amplitude = goal_state.getX() - start_state.getX();
    // Original formulation
    /*
    next_state.setXdd(
       (alpha * (beta * (goal_state.getX() - current_state.getX()) - current_state.getXd()*tau)
        + forcing_term * amplitude + spatial_coupling) / (tau * tau)
    );*/

    // Hoffman et al., ICRA 2009 formulation
    next_state.setXdd(
       (alpha * (goal_state.getX() - current_state.getX()) - beta * current_state.getXd()*tau
		//TODO:kipo BUG1 should be "-"
        //old:+ alpha * amplitude * canonical_state 
        - alpha * amplitude * canonical_state 
        + alpha * forcing_term + spatial_coupling) / (tau * tau)
    );

	fprintf(file_,"%lf\n",forcing_term);
    next_state.setXd(current_state.getXd() + next_state.getXdd() * dt);
    next_state.setX(current_state.getX() + next_state.getXd() * dt);
    //next_state.setTime(current_state.getTime() + dt);

    return next_state;
}

/**
 * Based on the given DMP state computes the desired/target value of the forcing function
 * Needed for learning the forcing function based on a given sequence of DMP states (trajectory)
 *
 * @param state DMP state
 * @param goal_state Goal state of the DMP
 * @param tau Time parameter
 */
double TransformSystemDiscrete::getDesiredForcingTerm(DMPState& state, DMPState& start_state, DMPState& goal_state, double tau)
{
    double amplitude = goal_state.getX() - start_state.getX();

    // Original formulation
    /*
    double fdesired = tau * tau * state.getXdd()
                    - alpha * (beta * (goal_state.getX() - state.getX()) - tau * state.getXd());
    fdesired /= amplitude;
    */

    // Hoffman et al., Pastor et al. ICRA 2009 formulation
    double fdesired = (tau * tau * state.getXdd() + beta * tau * state.getXd()) / alpha
            - (goal_state.getX() - state.getX()) + amplitude * state.getCanonicalState();
	fprintf(file_,"%lf\n",fdesired);
    return fdesired;
}

TransformSystemDiscrete::~TransformSystemDiscrete()
{
	fclose(file_);
}

}
