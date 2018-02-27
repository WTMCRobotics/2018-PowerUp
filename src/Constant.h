/*
 * Constant.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Team 6101
 */

#ifndef SRC_CONSTANT_H_
#define SRC_CONSTANT_H_


class Constant
{
public:
	static const int pidChannel = 0;

	static const int LeftLeaderID = 11;
	static const int LeftFollowerID = 12;
	static const int RightLeaderID = 14;
	static const int RightFollowerID = 13;

	static const int LiftLeaderID = 15;
	static const int LiftFollowerID = 16;

	static const int IntakeLeftPWM = 0;
	static const int IntakeRightPWM = 1;

	static const int PCM_ID = 0;
	static const int PCM_CHANNEL_CLAMP = 0;
	static const int PCM_CHANNEL_RELEASE = 0;

	static constexpr double pulsesPerRotationQuad = 2048 * 4;
	static constexpr double circumference = 6 * 3.14;

	static constexpr double tankDriveDeadbandVal = .15;
	static constexpr double arcadeDriveDeadbandVal = .2;

	// Target Calculation = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad
	static const int autonPositionDeadbandVal = 500;

	static constexpr double leftMotionVel = 5150;
	static constexpr double leftMotionAcc = 5150 / 2;

	static constexpr double rightMotionVel = 5150;
	static constexpr double rightMotionAcc = 5150 / 2;

	static constexpr double liftMotionVel = 0;
	static constexpr double liftMotionAcc = 0;
};


#endif /* SRC_CONSTANT_H_ */
