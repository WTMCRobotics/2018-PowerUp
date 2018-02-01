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

class PIDMotorOutput: public frc::PIDOutput
{
private:
	TalonSRX* leftMotor;
	TalonSRX* rightMotor;

public:

	PIDMotorOutput();
	PIDMotorOutput(TalonSRX* leftMotor, TalonSRX* rightMotor);

	virtual ~PIDMotorOutput();
};

#endif /* SRC_PIDMOTOROUTPUT_H_ */
