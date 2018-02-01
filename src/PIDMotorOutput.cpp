/*
 * PIDMotorOutput.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Team 6101
 */

#include <PIDMotorOutput.h>

	PIDMotorOutput()
	{

	}


PIDMotorOutput::PIDMotorOutput(TalonSRX* left, TalonSRX* right)
{
	leftMotor = left;
	rightMotor = right;
}

PIDMotorOutput::~PIDMotorOutput() {
	// TODO Auto-generated destructor stub
}

PIDMotorOutput::pidWrite(double output)
{
	left.
}
