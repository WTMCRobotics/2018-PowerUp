/*
 * PIDMotorOutput.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Team 6101
 */

#include <PIDMotorOutput.h>

PIDMotorOutput::PIDMotorOutput()
{

}


PIDMotorOutput::PIDMotorOutput(TalonSRX* left, TalonSRX* right)
{
	pLeftMotor = left;
	pRightMotor = right;
}

PIDMotorOutput::~PIDMotorOutput()
{
	// TODO Auto-generated destructor stub
}

//void PIDMotorOutput::pidWrite(double value)
//{
//	pLeftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -value);
//	pRightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, value);
//}
