/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <VictorSP.h>
#include <Joystick.h>
#include <AHRS.h>
#include "Constant.h"
#include "PIDMotorOutput.h"
#include "PIDGyroSource.h"
#include "PIDController.h"
#include <SerialPort.h>
#include <AnalogInput.h>
#include <Ultrasonic.h>
#include <DoubleSolenoid.h>
#include <Compressor.h>
#include <Timer.h>

#define JOYSTICK_CODRIVER
//#define GUITAR_CODRIVER

#define I2C_SLAVE_ADR 0x08 // ADXL345 I2C device address

//#define JOYSTICK_DRIVER
#define XBOX_DRIVER

enum angleMode{
	SMALL, MEDIUM, LARGE
};

enum driveMode {
	ARCADE, TANK
};
enum autonStates {
	START, TRAVERSE, DROP, TO_CUBE_INIT, TO_CUBE_TRAVERSE, TO_SCALE_INIT, TO_SCALE_TRAVERSE, DROP_SECOND, WAIT
} autonState;

enum traverseStates {
	MOVE, TURN, NEXT, DONE
} traverseState;

class Robot: public frc::TimedRobot {
public:
	void RobotInit() {

		setupMotor();
		// Auton Chooser
		m_chooser.AddDefault(autoDefault, autoDefault);
		m_chooser.AddObject(autoMiddle, autoMiddle);
		m_chooser.AddObject(autoLeft, autoLeft);
		m_chooser.AddObject(autoRight, autoRight);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		I2Channel = new I2C(I2C::kOnboard, I2C_SLAVE_ADR);

		// Reset Encoders
		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		liftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		lastLiftPosition = 0;

		// Stop Motors
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		liftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

		// Start traverse step at the beginning
		traverseStep = 0;

		scaleDone = false;
		noDrop = false;

		turnPID = SMALL;

		// Reset the gyroscope
//		gyro.Reset();
		gyro.ZeroYaw();
		while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01)) {waiting = true;}
		waiting = false;
//		ultrasonicSensor.SetAutomaticMode(true);
//		ultrasonicSensor.SetEnabled(true);

		secondsElapsed = 0;
		timerOverride = false;
		timer.Reset();

		// Update dashboard
		updateDashboard();

		// Start at the beginning of auton
		autonState = START;
	} // END of RobotInit()

	void I2CWrite(int data){
		I2Channel->Write(I2C_SLAVE_ADR, data);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */

	// Priority Based on Line Up
	// 		Middle Line Up = Switch (either side), grab another cube
	// 		Left Line Up = Switch, Scale, Only Cross Line (grab cube?)
	//		Right Line Up = Switch, Scale, Only Cross Line (grab cube?)
	void AutonomousInit() override {
		// Start at the beginning of auton
		autonState = START;
		timer.Start();

		// Get the color sides from field
		secondsElapsed = timer.Get();
		colorSides = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		while(colorSides.length() < 2 && secondsElapsed < 5)
		{
			colorSides = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			secondsElapsed = timer.Get();
		}

		timer.Stop();
		if(colorSides.length() < 2)
		{
			timerOverride = true;
		} else {
			timerOverride = false;
		}

		switch (turnPID) {
			case SMALL: pidSmallAngle.SetSetpoint(0); break;
			case MEDIUM: pidMediumAngle.SetSetpoint(0); break;
			case LARGE: pidLargeAngle.SetSetpoint(0); break;
		}

		// Get the auton mode from dashboard
		m_autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == autoMiddle) {
			// Lined up in the middle
			// if switch is on the left
			if (colorSides[0] == 'L') {

				// Drop Switch Left
				moveVector.push_back(10);
				turnVector.push_back(-45);

				moveVector.push_back(80.787);
				turnVector.push_back(0);

				moveVector.push_back(34.065);
				turnVector.push_back(181);			// Don't turn
			} // END of if switch is on the left
			  // else if the switch is on the right
			else if(colorSides[0] == 'R') {

				// Drop Switch Right
				moveVector.push_back(10);
				turnVector.push_back(45);

				moveVector.push_back(73.362);
				turnVector.push_back(0);

				moveVector.push_back(39.315);
				turnVector.push_back(181);			// Don't turn
			} // END of else the switch is on the right
			else
			{
				noDrop = true;
			}
		}
		// Lined up on the left
		else if (m_autoSelected == autoLeft) {
			if(colorSides[0] == 'L') {

				// Drop Switch Side
				moveVector.push_back(148.595);
				turnVector.push_back(90);
				moveVector.push_back(18.905);
				turnVector.push_back(181);
			}
			else if(colorSides[1] == 'L') {

				// Drop Scale
				moveVector.push_back(275.02);
				turnVector.push_back(45);
				moveVector.push_back(15.427);
				turnVector.push_back(181); // Don't turn

				scaleDone = true;
			}
			else
			{
				// Cross Line
				moveVector.push_back(130);
				turnVector.push_back(181);

				noDrop = true;
			}
		}
		// Lined up on the right
		else if (m_autoSelected == autoRight) {
			if(colorSides[0] == 'R') {

				// Drop Switch Side
				moveVector.push_back(148.595);
				turnVector.push_back(-90);
				moveVector.push_back(18.905);
				turnVector.push_back(181);
			}
			else if(colorSides[1] == 'R') {

				// Drop Scale
				moveVector.push_back(275.02);
				turnVector.push_back(-45);
				moveVector.push_back(15.427);
				turnVector.push_back(181); // Don't turn

				scaleDone = true;
			}
			else
			{
				// Cross Line
				moveVector.push_back(130);
				turnVector.push_back(181);

				noDrop = true;
			}
		}
		// Default auto
		else if (m_autoSelected == autoDefault) {
//			moveVector.push_back(130);
//			turnVector.push_back(0);
			moveVector.push_back(100);
			turnVector.push_back(0);

			moveVector.push_back(0);
			turnVector.push_back(45);

			moveVector.push_back(5);
			turnVector.push_back(181);


			//moveVector.push_back(150);
			//turnVector.push_back(0);

			noDrop = true;
		} else {
			moveVector.push_back(130);
			turnVector.push_back(0);

			noDrop = true;
		} // END of switch that outlines traverse path

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		pidSmallAngle.SetInputRange(-180, 180);
		pidSmallAngle.SetOutputRange(-0.25, 0.25);
		pidSmallAngle.SetContinuous(true);
		pidSmallAngle.SetAbsoluteTolerance(2);

		pidMediumAngle.SetInputRange(-180, 180);
		pidMediumAngle.SetOutputRange(-0.25, 0.25);
		pidMediumAngle.SetContinuous(true);
		pidMediumAngle.SetAbsoluteTolerance(2);

		pidLargeAngle.SetInputRange(-180, 180);
		pidLargeAngle.SetOutputRange(-0.25, 0.25);
		pidLargeAngle.SetContinuous(true);
		pidLargeAngle.SetAbsoluteTolerance(2);

		// Reset Encoders
		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		liftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		gyro.ZeroYaw();
		//pidMediumAngle.SetPID(0.1, 0.0, 0.0);
		updateDashboard();
	} // END of AutonomousInit()

	void AutonomousPeriodic() {
		updateCompressor();
		switch (autonState) {
			case START:
				// Stop Motors
				leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
				rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

				// Move to traverse and start traverse at the beginning
				autonState = TRAVERSE;
				traverseState = MOVE;
				// Set the traverse step to 0
				traverseStep = 0;

				switch (turnPID) {
					case SMALL: if (pidSmallAngle.IsEnabled()) {
									pidSmallAngle.Disable();
								} break;
				   case MEDIUM: if (pidMediumAngle.IsEnabled()) {
									pidMediumAngle.Disable();
								} break;
					case LARGE: if (pidLargeAngle.IsEnabled()) {
									pidLargeAngle.Disable();
								} break;
				} break;
			case TRAVERSE:
				updateDashboard();
				traverse();
				// if traverse is DONE
				if (traverseState == DONE) {
					// if auto selected is default
					if (noDrop) {
						// Skip drop
						autonState = WAIT;
					}
					// else not default auton
					else {
						// Move to drop
						autonState = DROP;
					}
					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

					moveVector.clear();
					turnVector.clear();


				} // END of if traverse is DONE
				break;
			case DROP:
				traverseState = MOVE;
				autonState = TO_CUBE_INIT;
				break;
			case TO_CUBE_INIT:
				if(!scaleDone && colorSides[1] == 'L' && m_autoSelected == autoLeft)
				{
					moveVector.push_back(-8.595);
					turnVector.push_back(0);
					moveVector.push_back(69);
					turnVector.push_back(135);
					moveVector.push_back(20.193);
					turnVector.push_back(181);		// Don't turn
				}
				else if(!scaleDone && colorSides[1] == 'R' && m_autoSelected == autoRight)
				{
					moveVector.push_back(-8.595);
					turnVector.push_back(0);
					moveVector.push_back(69);
					turnVector.push_back(-135);
					moveVector.push_back(20.193);
					turnVector.push_back(181);		// Don't turn
				}

				autonState = TO_CUBE_TRAVERSE;
				break;
			case TO_CUBE_TRAVERSE:
				updateDashboard();
				if(moveVector.empty())
				{
					autonState = WAIT;
					break;
				}
				traverse();
				// if traverse is DONE
				if (traverseState == DONE) {
					// Move to drop
					autonState = TO_SCALE_INIT;

					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					moveVector.clear();
					turnVector.clear();

					traverseState = MOVE;
				} // END of if traverse is DONE
				break;
			case TO_SCALE_INIT:
				if(m_autoSelected == autoLeft)
				{
					moveVector.push_back(-54.368);
					turnVector.push_back(45);
					moveVector.push_back(35.021);
					turnVector.push_back(181);		// Don't turn
				}
				else if(m_autoSelected == autoRight)
				{
					moveVector.push_back(-54.368);
					turnVector.push_back(-45);
					moveVector.push_back(35.021);
					turnVector.push_back(181);		// Don't turn
				}
				break;
			case TO_SCALE_TRAVERSE:
				updateDashboard();
				if(moveVector.empty())
				{
					autonState = WAIT;
					break;
				}
				traverse();
				// if traverse is DONE
				if (traverseState == DONE) {
					// Move to drop
					autonState = DROP_SECOND;

					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					moveVector.clear();
					turnVector.clear();
				} // END of if traverse is DONE
				break;
			case DROP_SECOND:
				traverseState = MOVE;
				break;
			case WAIT:
				break;
		}
		updateDashboard();
	} // END of AutonomousPeriodic()

	void TeleopInit() {
		// Stop Motors
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		liftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

		// Reset Encoders
		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		liftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		lastLiftPosition = 0;

		switch (turnPID) {
			case SMALL:	 pidSmallAngle.Disable();
						 break;
		    case MEDIUM: pidMediumAngle.Disable();
						 break;
			case LARGE:	 pidLargeAngle.Disable();
						 break;
		}

		gyro.Reset();
		gyro.ZeroYaw();
		while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01)) {waiting = true;}
		waiting = false;

//		ultrasonicSensor.SetAutomaticMode(true);
//		ultrasonicSensor.SetEnabled(true);
		updateDashboard();
	}

	void traverse() {
		updateDashboard();
		switch (traverseState) {
			case MOVE:
				if (moveVector[traverseStep] == 0 || driveDistance(moveVector[traverseStep])) {
					traverseState = TURN;
					updateDashboard();
					leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
					rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					updateDashboard();
//								while (gyro.IsCalibrating()) {
//									Wait(0.005);
//									moving = true;
//									updateDashboard();
//								}
//								moving = false;
//								gyro.ZeroYaw();
//								while (!(gyro.GetYaw() < 0.15 && gyro.GetYaw() > -.15)) {
//									Wait(0.005);
//									waiting = true;
//									updateDashboard();
//								}
//								waiting = false;
					if((turnVector[traverseStep] <= 45 && turnVector[traverseStep] >= 0)
							|| (turnVector[traverseStep] >= -45 && turnVector[traverseStep] <= 0))
					{
						turnPID = SMALL;
						pidSmallAngle.Enable();
					}
					else if((turnVector[traverseStep] <= 110 && turnVector[traverseStep] > 45)
							|| (turnVector[traverseStep] >= -110 && turnVector[traverseStep] < -45))
					{
						turnPID = MEDIUM;
							pidMediumAngle.Enable();
					}
					else {
						turnPID = LARGE;
						pidLargeAngle.Enable();
					}

					inTurnDegrees = false;
				}
				break;
			case TURN:
				if (turnVector[traverseStep] == 181 || turnDegrees(turnVector[traverseStep])) {
					traverseState = NEXT;
				}
				break;

			case NEXT:
				for(unsigned int i = 0; i < moveVector.size(); i++)
				{
					std::cout << "Move: " << moveVector[i] << std::endl;
					std::cout << "Turn: " << turnVector[i] << std::endl;
				}

				if (traverseStep + 1 < moveVector.size()) {
					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
					rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
					traverseStep++;
					traverseState = MOVE;
				} else {
					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
					rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
					if (pidSmallAngle.IsEnabled()) {
						pidSmallAngle.Disable();
					}
					if (pidMediumAngle.IsEnabled()) {
						pidMediumAngle.Disable();
					}
					if (pidLargeAngle.IsEnabled()) {
						pidLargeAngle.Disable();
					}
					traverseState = DONE;
				}
				break;
			case DONE:
				break;
		}
		updateDashboard();
	}

	bool turnDegrees(double degrees) {

//					if (!(pidSmallAngle.IsEnabled())) {
//						gyro.ZeroYaw();
//						while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01)) {waiting = true;}
//						waiting = false;
//						pidSmallAngle.Enable();
//						pidSmallAngle.SetSetpoint(degrees);
//						return false;
//					} else {
		inTurnDegrees = true;
		updateDashboard();
		switch (turnPID) {
			case SMALL: if(pidSmallAngle.GetSetpoint() != degrees)
							pidSmallAngle.SetSetpoint(degrees);
						break;
			case MEDIUM: if(pidMediumAngle.GetSetpoint() != degrees)
							pidMediumAngle.SetSetpoint(degrees);
						break;
			case LARGE: if(pidLargeAngle.GetSetpoint() != degrees)
							pidLargeAngle.SetSetpoint(degrees);
						break;
		}
		updateDashboard();
		if (pidMediumAngle.IsEnabled() && pidMediumAngle.OnTarget()) {
			pidMediumAngle.Disable();
			inTurnDegrees = false;
			updateDashboard();
			return true;
		}
		else if(pidSmallAngle.IsEnabled() && pidSmallAngle.OnTarget())
		{
			pidSmallAngle.Disable();
			inTurnDegrees = false;
			updateDashboard();
			return true;
		}
		else if(pidLargeAngle.IsEnabled() && pidLargeAngle.OnTarget())
		{
			pidLargeAngle.Disable();
			inTurnDegrees = false;
			updateDashboard();
			return true;
		}
		else {
			inTurnDegrees = false;
			updateDashboard();
			return false;
		}
//		}
	}

	void TeleopPeriodic() {
		//Drive(driveMode::ARCADE);
		Drive(driveMode::TANK);
		updateCompressor();
		updateDashboard();

		switch(pixelPosition){
			case 0:				// off
				I2CWrite(111);
				break;
			case 1:				// Red
				I2CWrite(114);
				break;
			case 2:				// Green
				I2CWrite(103);
				break;
			case 3:				// Blue
				I2CWrite(98);
				break;
			case 4:				// Rainbow
				I2CWrite(117);
				break;
			case 5:				// Rainbow Cycle
				I2CWrite(99);
				break;
			case 6:				// Chase
				I2CWrite(104);
				break;
			case 7:				// Checkerboard
				I2CWrite(116);
				break;
			case 8:				// Breathe
				I2CWrite(112);
				break;
			case 9:
				I2CWrite(115);
				break;
		}
	}

	void TestPeriodic() {
	}

	void UpdateJoystickCodriver() {
#if defined (JOYSTICK_CODRIVER)
		coJoyY = joystickCodriver.GetY();
		clampOpen = joystickCodriver.GetTriggerPressed();
		topStick = joystickCodriver.GetPOV();
		tempButton = joystickCodriver.GetRawButton(5);

#elif defined (GUITAR_CODRIVER)

#endif

	}

	void UpdateJoystickArcade() {
#if defined(JOYSTICK_DRIVER)
		joyY = joystick1.GetY();
		joyX = -joystick1.GetX();
		joyZ = joystick1.GetZ();
#endif
	}



	void UpdateJoystickTank() {
#if defined(JOYSTICK_DRIVER)
		leftjoyY = joystick1.GetY();
		rightjoyY = joystick2.GetY();
#elif defined(XBOX_DRIVER)
		//If both triggers pressed
		if(!(xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) >0.1)  && !(xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand))) {
			//Arcade drive with right stick
			rightjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kRightHand);
			rightjoyX = xboxController.GetX(frc::GenericHID::JoystickHand::kRightHand);
		} else {
			//Tank drive both stick
			leftjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
			rightjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kRightHand);
		}

		//If right trigger pressed
		if((xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) >0.1)  && !(xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand))) {
			//Kinda slow down
			leftjoyY = leftjoyY / 1.5;
			rightjoyY = rightjoyY / 1.5;
		}
		//If both triggers pressed
		if((xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand)) >0.1 && (xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) >0.1)) {
			//Really slow down
			leftjoyY = leftjoyY / 3;
			rightjoyY = rightjoyY / 3;
		}
#endif
	}

	void Drive(driveMode mode) {
		UpdateJoystickCodriver();

		//Codriver stuff
		//Clamp control
		if (clampOpen) {
			//Do pneumatics stuff to open clamp
			intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
		} else {
			//Do pneumatics stuff to close clamp
			intakeSolenoid.Set(DoubleSolenoid::Value::kReverse);
		}

		//Intake control
		if (topStick >= 315 || topStick <= 45) {
			//slow for outtake
			clampWheelsTarget = 0.3;
		} else {
			//fast for intake
			clampWheelsTarget = 1;
		}

		//Lift control
		armTarget = coJoyY;


		//If we are in arcade drive
		if (mode == driveMode::ARCADE) {
			UpdateJoystickArcade();
			//If Z axis is moved and X and Y are not
			if (ArcadeDriveDeadband(joyZ) != 0 && TankDriveDeadband(joyY) == 0 && TankDriveDeadband(joyX) == 0) {
				//Spin robot on a dime
				if (joyZ > 0) {
					leftTarget = -joyZ;
					rightTarget = joyZ;
				} else {
					leftTarget = -joyZ;
					rightTarget = joyZ;
				}
			//if Y is less than 0
			} else if (ArcadeDriveDeadband(joyY) <= 0) {
				//Make it so that if it is pulled back and to the left, it goes back and to the left
				leftTarget = TankDriveDeadband(joyY + joyX);
				rightTarget = TankDriveDeadband(joyY - joyX);
			}
			//If Y is greater than 0
			else {
				//Make it so that if it is pushed forward and to the right, it goes forward and to the right
				leftTarget = TankDriveDeadband(joyY - joyX);
				rightTarget = TankDriveDeadband(joyY + joyX);
			}
		} else {
			//If we are in tank drive
			UpdateJoystickTank();

			//If neither triggers are pressed
			if (!(xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) >0.1)  && !(xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand))) {
				//if Y is less than 0
				if (ArcadeDriveDeadband(rightjoyY) <= 0) {
					//Make it so that if it is pulled back and to the left, it goes back and to the left
					leftTarget = TankDriveDeadband(rightjoyY - rightjoyX);
					rightTarget = TankDriveDeadband(rightjoyY + rightjoyX);
				} else { //If Y is greater than 0
					//Make it so that if it is pushed forward and to the right, it goes forward and to the right
					leftTarget = TankDriveDeadband(rightjoyY + rightjoyX);
					rightTarget = TankDriveDeadband(rightjoyY - rightjoyX);
				}

			} else { //Do normal tank drive
				leftTarget = TankDriveDeadband(leftjoyY);
				rightTarget = TankDriveDeadband(rightjoyY);
			}
		}
		//Square the absolute value of the left for ramping purposes
		if (leftTarget < 0) {
			leftTarget *= leftTarget;
			leftTarget = -leftTarget;
		} else {
			leftTarget *= leftTarget;
		}
		//Square the absolute value of the right for ramping purposes
		if (rightTarget < 0) {
			rightTarget *= rightTarget;
			rightTarget = -rightTarget;
		} else {
			rightTarget *= rightTarget;
		}

		//Left motor move, negative value = forward
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftTarget);
		//Right motor move
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -rightTarget);

		//Intake Control
		if(topStick != -1) {
			intakeLeftMotor.Set(clampWheelsTarget);
			intakeRightMotor.Set(-clampWheelsTarget);
		} else {
			intakeLeftMotor.Set(0);
			intakeRightMotor.Set(0);
		}

		//Lift Control
		if(TankDriveDeadband(armTarget) == 0) {
			liftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, lastLiftPosition);
		}else if(tempButton)
		{
			liftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 1024 * 4);
		}
		else{
			liftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, armTarget);
			lastLiftPosition = liftLeader.GetSelectedSensorPosition(Constant::pidChannel);
		}
	}

	bool driveDistance(double inches) {
		updateDashboard();
		// inches / circumference = number of rotations
		// * pulsesPerRotationQuad = number of pulses in one rotation
		// targetEncPos = position encoder should read
		targetEncPos = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad;

		if (AutonPositionDeadband(leftLeader.GetSelectedSensorPosition(Constant::pidChannel), targetEncPos)) {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			return true;
		}

//					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::LeftLeaderID);
//					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetEncPos);

		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);

//					// Forward = positive encoder position for left
//					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetEncPos);
//					// Forward = negative encoder position for right
//					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, -targetEncPos);

		return false;
	}

	void setupMotor() {
		//Left motor setup
		leftLeader.ClearStickyFaults(0);
		leftLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		leftLeader.ConfigNominalOutputForward(0, 0);
		leftLeader.ConfigNominalOutputReverse(0, 0);
		leftLeader.ConfigPeakOutputForward(1, 0);
		leftLeader.ConfigPeakOutputReverse(-1, 0);
		leftLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftLeader.ConfigMotionCruiseVelocity(Constant::leftMotionVel, 0);
		leftLeader.ConfigMotionAcceleration(Constant::leftMotionAcc, 0);
		leftLeader.SetSensorPhase(false);
		leftLeader.SetInverted(false);

		leftFollower.ConfigNominalOutputForward(0, 0);
		leftFollower.ConfigNominalOutputReverse(0, 0);
		leftFollower.ConfigPeakOutputForward(1, 0);
		leftFollower.ConfigPeakOutputReverse(-1, 0);
		leftFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftFollower.ConfigMotionCruiseVelocity(Constant::leftMotionVel, 0);
		leftFollower.ConfigMotionAcceleration(Constant::leftMotionAcc, 0);
		leftFollower.SetSensorPhase(false);
		leftFollower.SetInverted(false);


		//Right motor setup
		rightLeader.ClearStickyFaults(0);
		rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightLeader.ConfigMotionCruiseVelocity(Constant::rightMotionVel, 0);
		rightLeader.ConfigMotionAcceleration(Constant::rightMotionAcc, 0);
		rightLeader.SetSensorPhase(false);
		rightLeader.SetInverted(true);

		rightFollower.ConfigNominalOutputForward(0, 0);
		rightFollower.ConfigNominalOutputReverse(0, 0);
		rightFollower.ConfigPeakOutputForward(1, 0);
		rightFollower.ConfigPeakOutputReverse(-1, 0);
		rightFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightFollower.ConfigMotionCruiseVelocity(Constant::rightMotionVel, 0);
		rightFollower.ConfigMotionAcceleration(Constant::rightMotionAcc, 0);
		rightFollower.SetSensorPhase(false);
		rightFollower.SetInverted(true);

		// Lift Setup
		liftLeader.ClearStickyFaults(0);
		liftLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		liftLeader.ConfigNominalOutputForward(0, 0);
		liftLeader.ConfigNominalOutputReverse(0, 0);
		liftLeader.ConfigPeakOutputForward(1, 0);
		liftLeader.ConfigPeakOutputReverse(-1, 0);
		liftLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		liftLeader.SetSensorPhase(false);
		liftLeader.SetInverted(false);

		liftFollower.ConfigNominalOutputForward(0, 0);
		liftFollower.ConfigNominalOutputReverse(0, 0);
		liftFollower.ConfigPeakOutputForward(1, 0);
		liftFollower.ConfigPeakOutputReverse(-1, 0);
		liftFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		liftFollower.SetSensorPhase(false);
		liftFollower.SetInverted(true);

		// PID Setup
		leftLeader.Config_kP(Constant::pidChannel, .09, 0);
		leftLeader.Config_kI(Constant::pidChannel, 0, 0);
		leftLeader.Config_kD(Constant::pidChannel, 0, 0);
		leftLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		rightLeader.Config_kP(Constant::pidChannel, .09, 0);
		rightLeader.Config_kI(Constant::pidChannel, 0, 0);
		rightLeader.Config_kD(Constant::pidChannel, 0, 0);
		rightLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		liftLeader.Config_kP(Constant::pidChannel, .1, 0);
		liftLeader.Config_kI(Constant::pidChannel, 0, 0);
		liftLeader.Config_kD(Constant::pidChannel, 0, 0);
		liftLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::LeftLeaderID);
		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::RightLeaderID);
		liftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::LiftLeaderID);
	}

	void updateDashboard() {
		frc::SmartDashboard::PutNumber("POV Codriver", joystickCodriver.GetPOV());
		frc::SmartDashboard::PutBoolean("Compressor On", compressor.Enabled());
		frc::SmartDashboard::PutBoolean("Switch Valve", compressor.GetPressureSwitchValue());

		frc::SmartDashboard::PutNumber("Left Enc Pos", leftLeader.GetSelectedSensorPosition(Constant::Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Error", leftLeader.GetClosedLoopError(Constant::pidChannel));
        //				frc::SmartDashboard::PutNumber("Left Target", leftLeader.GetClosedLoopTarget(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Enc Vel", leftLeader.GetSelectedSensorVelocity(Constant::pidChannel));

		frc::SmartDashboard::PutNumber("Right Enc Pos", rightLeader.GetSelectedSensorPosition(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Error", rightLeader.GetClosedLoopError(Constant::pidChannel));
        //				frc::SmartDashboard::PutNumber("Right Target", rightLeader.GetClosedLoopTarget(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Enc Vel", rightLeader.GetSelectedSensorVelocity(Constant::pidChannel));

		frc::SmartDashboard::PutNumber("Lift Enc Pos", liftLeader.GetSelectedSensorPosition(Constant::Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Lift Enc Err", liftLeader.GetClosedLoopError(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Last Lift Position", lastLiftPosition);
		frc::SmartDashboard::PutBoolean("Temp Button", tempButton);

		frc::SmartDashboard::PutNumber("Angle", gyro.GetYaw());
		frc::SmartDashboard::PutBoolean("Waiting to Zero", waiting);

		switch (autonState) {
			case START:
				frc::SmartDashboard::PutString("Auton State", "Start");
				break;
			case TRAVERSE:
				frc::SmartDashboard::PutString("Auton State", "Traverse");
				break;
			case DROP:
				frc::SmartDashboard::PutString("Auton State", "1st Drop");
				break;
			case TO_CUBE_INIT:
				frc::SmartDashboard::PutString("Auton State", "Cube Init");
				break;
			case TO_CUBE_TRAVERSE:
				frc::SmartDashboard::PutString("Auton State", "Cube Traverse");
				break;
			case TO_SCALE_INIT:
				frc::SmartDashboard::PutString("Auton State", "Scale Init");
				break;
			case TO_SCALE_TRAVERSE:
				frc::SmartDashboard::PutString("Auton State", "Scale Traverse");
				break;
			case DROP_SECOND:
				frc::SmartDashboard::PutString("Auton State", "2nd Drop");
				break;
			case WAIT:
				frc::SmartDashboard::PutString("Auton State", "Wait");
				break;
		}

		switch (traverseState) {
			case MOVE:
				frc::SmartDashboard::PutString("Traverse State", "Move");
				break;
			case TURN:
				frc::SmartDashboard::PutString("Traverse State", "Turn");
				break;
			case NEXT:
				frc::SmartDashboard::PutString("Traverse State", "Next");
				break;
			case DONE:
				frc::SmartDashboard::PutString("Traverse State", "Done");
				break;
		}

		frc::SmartDashboard::PutNumber("Traverse Step", traverseStep);
		if (moveVector.size() > 0)
			frc::SmartDashboard::PutNumber("Auton Move Command", moveVector[traverseStep]);
		else
			frc::SmartDashboard::PutNumber("Auton Move Command", -1);
		if (turnVector.size() > 0)
			frc::SmartDashboard::PutNumber("Auton Turn Command", turnVector[traverseStep]);
		else
			frc::SmartDashboard::PutNumber("Auton Turn Command", -1);

		frc::SmartDashboard::PutBoolean("Timer Override", timerOverride);

		switch (turnPID) {
			case SMALL: frc::SmartDashboard::PutBoolean("PIDIsOnTarget", pidSmallAngle.OnTarget());
						frc::SmartDashboard::PutNumber("PIDTarget", pidSmallAngle.GetSetpoint());
						frc::SmartDashboard::PutBoolean("IsEnabled", pidSmallAngle.IsEnabled());
						frc::SmartDashboard::PutBoolean("Turn Degrees Function", inTurnDegrees);
						frc::SmartDashboard::PutNumber("PIDError", pidSmallAngle.GetError());
						break;
			case MEDIUM: frc::SmartDashboard::PutBoolean("PIDIsOnTarget", pidMediumAngle.OnTarget());
						 frc::SmartDashboard::PutNumber("PIDTarget", pidMediumAngle.GetSetpoint());
						 frc::SmartDashboard::PutBoolean("IsEnabled", pidMediumAngle.IsEnabled());
						 frc::SmartDashboard::PutBoolean("Turn Degrees Function", inTurnDegrees);
						 frc::SmartDashboard::PutNumber("PIDError", pidMediumAngle.GetError());
						 break;
			case LARGE:  frc::SmartDashboard::PutBoolean("PIDIsOnTarget", pidLargeAngle.OnTarget());
						 frc::SmartDashboard::PutNumber("PIDTarget", pidLargeAngle.GetSetpoint());
						 frc::SmartDashboard::PutBoolean("IsEnabled", pidLargeAngle.IsEnabled());
						 frc::SmartDashboard::PutBoolean("Turn Degrees Function", inTurnDegrees);
						 frc::SmartDashboard::PutNumber("PIDError", pidLargeAngle.GetError());
						 break;
		}

		if(pidMediumAngle.IsEnabled())
		{
			frc::SmartDashboard::PutBoolean("Medium PID", true);
			frc::SmartDashboard::PutNumber("P Gain", pidMediumAngle.GetP());
			frc::SmartDashboard::PutNumber("I Gain", pidMediumAngle.GetI());
			frc::SmartDashboard::PutNumber("D Gain", pidMediumAngle.GetD());
		}
		else
			frc::SmartDashboard::PutBoolean("Medium PID", false);

		if(pidSmallAngle.IsEnabled())
		{
			frc::SmartDashboard::PutBoolean("Small PID", true);
			frc::SmartDashboard::PutNumber("P Gain", pidSmallAngle.GetP());
			frc::SmartDashboard::PutNumber("I Gain", pidSmallAngle.GetI());
			frc::SmartDashboard::PutNumber("D Gain", pidSmallAngle.GetD());
		}
		else
			frc::SmartDashboard::PutBoolean("Small PID", false);

		if(pidLargeAngle.IsEnabled())
		{
			frc::SmartDashboard::PutBoolean("Large PID", true);
			frc::SmartDashboard::PutNumber("P Gain", pidLargeAngle.GetP());
			frc::SmartDashboard::PutNumber("I Gain", pidLargeAngle.GetI());
			frc::SmartDashboard::PutNumber("D Gain", pidLargeAngle.GetD());
		}
		else
			frc::SmartDashboard::PutBoolean("Large PID", false);

		if(!pidMediumAngle.IsEnabled() && !pidSmallAngle.IsEnabled() && !pidSmallAngle.IsEnabled())
		{
			frc::SmartDashboard::PutNumber("P Gain", -1);
			frc::SmartDashboard::PutNumber("I Gain", -1);
			frc::SmartDashboard::PutNumber("D Gain", -1);
		}

		frc::SmartDashboard::PutNumber("Cube Sensor Value", cubeSensor.GetValue());
		frc::SmartDashboard::PutNumber("Cube Sensor Voltage", cubeSensor.GetVoltage());

//		frc::SmartDashboard::PutBoolean("Ultrasonic Enabled", ultrasonicSensor.IsEnabled());
//		frc::SmartDashboard::PutNumber("Ultrasonic", ultrasonicSensor.GetRangeInches());

		frc::SmartDashboard::PutBoolean("Beam Break", beamBreakSensor.Get());
	}

	void updateCompressor(void)
	{
		// if not enough pressure
		if(!compressor.GetPressureSwitchValue()) {
			// Start compressor
			compressor.Start();
		}
		// if enough pressure
		else {
			// Stop compressor
			compressor.Stop();
		}
	} // END of UpdateCompressor() function

	bool AutonPositionDeadband(double value, int target) {
		if (fabs(target - value) < Constant::autonPositionDeadbandVal) {
			return true;
		} else {
			return false;
		}
	}

	double TankDriveDeadband(double value) {
		if (value <= Constant::tankDriveDeadbandVal && value >= -Constant::tankDriveDeadbandVal) {
			return 0;
		} else {
			return value;
		}
	}

	double ArcadeDriveDeadband(double value) {
		if (value <= Constant::arcadeDriveDeadbandVal && value >= -Constant::arcadeDriveDeadbandVal) {
			return 0;
		} else {
			return value;
		}
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string autoDefault = "Default";
	const std::string autoMiddle = "Middle";
	const std::string autoLeft = "Left";
	const std::string autoRight = "Right";
	std::string m_autoSelected;

	std::vector<double> moveVector;
	std::vector<double> turnVector;

	bool scaleDone = false;
	bool noDrop = false;

	I2C *I2Channel;
	int pixelPosition = 0;

	Timer timer;
	double secondsElapsed = 0;
	bool timerOverride = false;

	TalonSRX leftLeader { Constant::LeftLeaderID };
	TalonSRX leftFollower { Constant::LeftFollowerID };
	TalonSRX rightLeader { Constant::RightLeaderID };
	TalonSRX rightFollower { Constant::RightFollowerID };

	TalonSRX liftLeader{Constant::LiftLeaderID};
	TalonSRX liftFollower{Constant::LiftFollowerID};
	double lastLiftPosition;
	frc::DoubleSolenoid intakeSolenoid {Constant::PCM_ID, Constant::PCM_CHANNEL_CLAMP, Constant::PCM_CHANNEL_RELEASE};
	frc::Compressor compressor{Constant::PCM_ID};

	VictorSP intakeLeftMotor{Constant::IntakeLeftPWM};
	VictorSP intakeRightMotor{Constant::IntakeRightPWM};

#if defined(JOYSTICK_DRIVER)
	Joystick joystick1 { 0 };		    // Arcade and Left Tank
	Joystick joystick2 { 1 };			// Right Tank
#endif
	PIDMotorOutput pidMotorOutput { &leftLeader, &rightLeader };
	PIDGyroSource pidGyroSource { &gyro };
	PIDController pidSmallAngle { .0125, 0, 0.01, &pidGyroSource, &pidMotorOutput, 0.02 };  //trying for 45 degrees
    //	    PIDController pidSmallAngle { .00822, 0, 0, &pidGyroSource, &pidMotorOutput, 0.02 };
	PIDController pidMediumAngle { .01094, 0, 0.01, &pidGyroSource, &pidMotorOutput, 0.02 }; //trying for 90 degrees
	PIDController pidLargeAngle { .004, 0, 0.02, &pidGyroSource, &pidMotorOutput, 0.02 }; //trying for 135 degrees
	//      PIDController pidController { .00602, 0, 0, &pidGyroSource, &pidMotorOutput, 0.02 };
#if defined(JOYSTICK_CODRIVER)
	Joystick joystickCodriver { 1 };
#elif defined(GUITAR_CODRIVER)
	XboxController guitar { 1 };
#endif
	double armTarget;
	bool clampOpen;
	double coJoyY;
	double joyX;
	double joyY;
	double joyZ;
	double leftjoyY;
	double rightjoyY;
	double rightjoyX;
#if defined(XBOX_DRIVER)
	XboxController xboxController{0};
#endif
	int topStick;
	double clampWheelsTarget;
	double leftTarget;
	double rightTarget;
	double targetEncPos;
	bool waiting;
	bool inTurnDegrees = false;
	unsigned int traverseStep = 0;
	angleMode turnPID = MEDIUM;
	AHRS gyro { I2C::Port::kMXP };

	AnalogInput cubeSensor{0};
	// Ultrasonic ultrasonicSensor{2, 2, Ultrasonic::kInches};
	DigitalInput beamBreakSensor{2};
	bool tempButton;
	ctre::phoenix::motorcontrol::StickyFaults leftFaults;
	ctre::phoenix::motorcontrol::StickyFaults rightFaults;

	std::string colorSides;
};

START_ROBOT_CLASS(Robot)
