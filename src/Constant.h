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

	static const int LeftLeaderID = 1;
	static const int LeftFollowerID = 2;
	static const int RightLeaderID = 4;
	static const int RightFollowerID = 3;

	static constexpr double pulsesPerRotationQuad = 2048 * 4;
	static constexpr double circumference = 6 * 2 * 3.14;
};


#endif /* SRC_CONSTANT_H_ */
