/*
 * PIDMotorOutput.h
 *
 *  Created on: Jan 31, 2018
 *      Author: Team 6101
 */

#ifndef SRC_PIDMOTOROUTPUT_H_
#define SRC_PIDMOTOROUTPUT_H_

#include <LiveWindow/LiveWindow.h>
#include <ctre/Phoenix.h>
#include "PIDController.h"

class PIDMotorOutput : public frc::PIDOutput
{
private:
	TalonSRX* pLeftMotor;
	TalonSRX* pRightMotor;

public:

	PIDMotorOutput();
	PIDMotorOutput(TalonSRX* leftMotor, TalonSRX* rightMotor);
	virtual ~PIDMotorOutput();

	void pidWrite(double value)
	{
		pLeftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -value);
		pRightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, value);
	}


};

#endif /* SRC_PIDMOTOROUTPUT_H_ */
