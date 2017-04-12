#ifndef IMPEDANCE_COUPLING 
#define IMPEDANCE_COUPLING

#include "SpatialCoupling.h"
#include "DMPState.h"

using namespace dmp;
class ImpedanceCoupling:public dmp::SpatialCoupling
{
	public:
		ImpedanceCoupling();
		virtual ~ImpedanceCoupling();
		double getValue(DMPState& state) ;
		double getValue(void) ;
		void   setValue(double value) ;

	private:
		double  value_;
};

#endif /* ifndef  */
