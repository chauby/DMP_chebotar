
#include "ImpedanceCoupling.h"
#include <stdio.h>

ImpedanceCoupling::ImpedanceCoupling()
{
}

ImpedanceCoupling::~ImpedanceCoupling()
{
}

/**
 * @brief 获取相应的偶合项目
 * @param state
 * @return 
 */
double ImpedanceCoupling::getValue(DMPState& state) 
{
	return value_;
}

double ImpedanceCoupling::getValue(void) 
{
	return value_;
}

void ImpedanceCoupling::setValue(double value) 
{
	value_ = value;
}

