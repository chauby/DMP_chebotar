/*
 * DMPState.h
 *
 *  Data object that holds variables that can be needed for estimating
 *  the current DMP state including position, velocity, acceleration,
 *  canonical state and time
 *
 *  Created on: Dec 12, 2014
 *  Author: chebotar
 */

#ifndef DMP_STATE_H
#define DMP_STATE_H

namespace dmp{

class DMPState{

private:
	double x;
	double xd;
	double xdd;
	double canonical_state;
	double time;

public:
	double getX(){ return x; }
	double getXd() { return xd; }
	double getXdd() { return xdd; }
	double getCanonicalState() { return canonical_state; }
	double getTime(){ return time; }

	void setX(double new_x) { x = new_x; }
	void setXd(double new_xd) { xd = new_xd; }
	void setXdd(double new_xdd) { xdd = new_xdd; }
	void setCanonicalState(double new_canonical_state) { canonical_state = new_canonical_state; }
	void setTime(double new_time){ time = new_time; }

	DMPState():
		x(0), xd(0), xdd(0), canonical_state(0), time(0){};

	DMPState(double x, double xd, double xdd):
		x(x), xd(xd), xdd(xdd), canonical_state(0), time(0) {};

	DMPState(const DMPState& other)
	{
		x = other.x;
		xd = other.xd;
		xdd = other.xdd;
		canonical_state = other.canonical_state;
		time = other.time;
	}

	DMPState& operator=(const DMPState& other)
	{
		x = other.x;
		xd = other.xd;
		xdd = other.xdd;
		canonical_state = other.canonical_state;
		time = other.time;
		return *this;
	}

};
}
#endif
