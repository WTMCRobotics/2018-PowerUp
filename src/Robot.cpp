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
#include <Joystick.h>
#include <AHRS.h>
#include "Constant.h"
#include "PIDMotorOutput.h"
#include "PIDGyroSource.h"
#include "PIDController.h"
#include <SerialPort.h>

enum driveMode {
	ARCADE, TANK
};
enum autonStates {
	START, TRAVERSE, DROP, WAIT
} autonState;

enum traverseStates {
	MOVE, TURN, NEXT, DONE
} traverseState;

class Robot: public frc::TimedRobot {
public:
	void RobotInit() {
		// Auton Chooser
		m_chooser.AddDefault(autoDefault, autoDefault);
		m_chooser.AddObject(autoMiddle, autoMiddle);
		m_chooser.AddObject(autoLeft, autoLeft);
		m_chooser.AddObject(autoRight, autoRight);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		// Stop Motors
		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		// Start traverse step at the beginning
		traverseStep = 0;

		// Reset the gyroscope
//		gyro.Reset();
		gyro.ZeroYaw();
		while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01)) {waiting = true;}
		waiting = false;

		// Update dashboard
		updateDashboard();

		// Start at the beginning of auton
		autonState = START;
	} // END of RobotInit()

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

		// Get the color sides from field
		colorSides = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		pidController.SetSetpoint(0);

		// Get the auton mode from dashboard
		m_autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == autoMiddle) {
			// Lined up in the middle
			// if switch is on the left
			if (colorSides[0] == 'L') {
				moveVector.push_back(10);
				turnVector.push_back(-23.38);

				moveVector.push_back(130.73);
				turnVector.push_back(0);

				moveVector.push_back(10);
				turnVector.push_back(181);			// Don't turn
			} // END of if switch is on the left
			  // else if the switch is on the right
			else {
				moveVector.push_back(10);
				turnVector.push_back(23.38);

				moveVector.push_back(130.73);
				turnVector.push_back(0);

				moveVector.push_back(10);
				turnVector.push_back(181);			// Don't turn
			} // END of else the switch is on the right
		}
		// Lined up on the left
		else if (m_autoSelected == autoLeft) {
			if(colorSides[0] == 'L'){
				moveVector.push_back(168);
				turnVector.push_back(90);
			}
		}
		// Lined up on the right
		else if (m_autoSelected == autoRight) {
			if(colorSides[0] == 'R'){
				moveVector.push_back(168);
				turnVector.push_back(-90);
			}
		}
		// Default auto
		else if (m_autoSelected == autoDefault) {
			moveVector.push_back(130);
			turnVector.push_back(0);
		} else {
			moveVector.push_back(130);
			turnVector.push_back(0);
		} // END of switch that outlines traverse path

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		pidController.SetInputRange(-180, 180);
		pidController.SetOutputRange(-0.5, 0.5);
		pidController.SetContinuous(true);
		pidController.SetAbsoluteTolerance(1);

		gyro.ZeroYaw();
		//pidController.SetPID(0.1, 0.0, 0.0);
		updateDashboard();
	} // END of AutonomousInit()

	void AutonomousPeriodic() {
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

				// Disable PID
				if (pidController.IsEnabled()) {
					pidController.Disable();
				} // END of disabling PID
				break;
			case TRAVERSE:
				updateDashboard();
				traverse();
				// if traverse is DONE
				if (traverseState == DONE) {
					// if auto selected is default
					if (m_autoSelected == autoDefault)
						// Skip drop
						autonState = WAIT;
					// else not default auton
					else
						// Move to drop
						autonState = DROP;

					leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					moveVector.clear();
					turnVector.clear();
				} // END of if traverse is DONE
				break;
			case DROP:
				break;
			case WAIT:
				break;
		}
		updateDashboard();
	} // END of AutonomousPeriodic()

	void TeleopInit() {

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		pidController.Disable();

		gyro.Reset();
		gyro.ZeroYaw();
		while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01)) {waiting = true;}
		waiting = false;

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
					updateDashboard();
//					while (gyro.IsCalibrating()) {
//						Wait(0.005);
//						moving = true;
//						updateDashboard();
//					}
//					moving = false;
//					gyro.ZeroYaw();
//					while (!(gyro.GetYaw() < 0.15 && gyro.GetYaw() > -.15)) {
//						Wait(0.005);
//						waiting = true;
//						updateDashboard();
//					}
//					waiting = false;
					pidController.Enable();
					inTurnDegrees = false;
				}
				break;
			case TURN:
				if (turnVector[traverseStep] == 181 || turnDegrees(turnVector[traverseStep])) {
					traverseState = NEXT;
				}
				break;

				//				switch (traverseStep) {
				//				case 0:
				//					if (turnDegrees(90))
				//					{
				//						traverseStep++;
				//						gyro.ZeroYaw();
				//						while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01))
				//							{waiting = true; updateDashboard();}
				//						waiting = false;
				//					}
				//					break;
				//				case 1:
				//					std::cout << "traverseStep is 1\n";
				//					if(turnDegrees(-90))
				//						{autonState = WAIT;}
				//					break;
				//				}
				//				break;
			case NEXT:
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
					if (pidController.IsEnabled()) {
						pidController.Disable();
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

//		if (!(pidController.IsEnabled())) {
//			gyro.ZeroYaw();
//			while(!(gyro.GetYaw() < 0.01 && gyro.GetYaw() > -.01)) {waiting = true;}
//			waiting = false;
//			pidController.Enable();
//			pidController.SetSetpoint(degrees);
//			return false;
		//} else {
		inTurnDegrees = true;
		updateDashboard();
		if(pidController.GetSetpoint() != degrees)
			pidController.SetSetpoint(degrees);

		updateDashboard();

		if (pidController.OnTarget()) {
			pidController.Disable();
			inTurnDegrees = false;
			updateDashboard();
			return true;
		} else {
			inTurnDegrees = false;
			updateDashboard();
			return false;
		}
//		}
	}

	void TeleopPeriodic() {
		//Drive(driveMode::ARCADE);
		Drive(driveMode::TANK);
		updateDashboard();
	}

	void TestPeriodic() {
	}

	void UpdateJoystickArcade() {
		joyY = joystick1.GetY();
		joyX = -joystick1.GetX();
		joyZ = joystick1.GetZ();

	}

	void UpdateJoystickTank() {
		leftjoyY = joystick1.GetY();
		rightjoyY = joystick2.GetY();
	}

	void Drive(driveMode mode) {

		if (mode == driveMode::ARCADE) {

			UpdateJoystickArcade();
			if (ArcadeDriveDeadband(joyZ) != 0 && TankDriveDeadband(joyY) == 0 && TankDriveDeadband(joyX) == 0) {
				if (joyZ > 0) {
					leftTarget = -joyZ;
					rightTarget = joyZ;
				} else {
					leftTarget = -joyZ;
					rightTarget = joyZ;
				}
			} else if (ArcadeDriveDeadband(joyY) <= 0) {
				leftTarget = TankDriveDeadband(joyY + joyX);
				rightTarget = TankDriveDeadband(joyY - joyX);
			}

			else {
				leftTarget = TankDriveDeadband(joyY - joyX);
				rightTarget = TankDriveDeadband(joyY + joyX);
			}
		} else {
			UpdateJoystickTank();
			leftTarget = TankDriveDeadband(leftjoyY);
			rightTarget = TankDriveDeadband(rightjoyY);
		}

		if (leftTarget < 0) {
			leftTarget *= leftTarget;
			leftTarget = -leftTarget;
		} else {
			leftTarget *= leftTarget;
		}

		if (rightTarget < 0) {
			rightTarget *= rightTarget;
			rightTarget = -rightTarget;
		} else {
			rightTarget *= rightTarget;
		}
		//Left motor move, negative value = forward
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftTarget);

		//Right motor move
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightTarget);
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
		// Forward = positive encoder position for left
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetEncPos);
		// Forward = negative encoder position for right
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, -targetEncPos);

		return false;
	}

	void SetupMotor() {
		//Left motor setup
		leftLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		leftLeader.SetSensorPhase(true);
		leftLeader.SetInverted(false);
		leftLeader.ConfigNominalOutputForward(0, 0);
		leftLeader.ConfigNominalOutputReverse(0, 0);
		leftLeader.ConfigPeakOutputForward(1, 0);
		leftLeader.ConfigPeakOutputReverse(-1, 0);
		leftLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftFollower.SetSensorPhase(true);
		leftFollower.SetInverted(false);

		// Set Left PID
		// Values were tested using Web Interface
		leftLeader.Config_kP(Constant::pidChannel, .5, 0);
		leftLeader.Config_kI(Constant::pidChannel, .009, 0);
		leftLeader.Config_kD(Constant::pidChannel, 13, 0);
		leftLeader.Config_IntegralZone(Constant::pidChannel, 100, 0);

		rightLeader.Config_kP(Constant::pidChannel, .5, 0);
		rightLeader.Config_kI(Constant::pidChannel, .009, 0);
		rightLeader.Config_kD(Constant::pidChannel, 13, 0);
		rightLeader.Config_IntegralZone(Constant::pidChannel, 100, 0);

		leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, 1);
		leftFollower.SetSensorPhase(true);
		leftFollower.SetInverted(false);

		//Right motor setup
		rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder,
				Constant::pidChannel, 0);
		rightLeader.SetSensorPhase(false);
		rightLeader.SetInverted(true);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, 4);
		rightFollower.SetSensorPhase(false);
		rightFollower.SetInverted(true);
	}

	void updateDashboard() {
//		frc::SmartDashboard::PutNumber("Left Enc Pos",
//				leftLeader.GetSelectedSensorPosition(Constant::Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Error", leftLeader.GetClosedLoopError(Constant::pidChannel));
//		frc::SmartDashboard::PutNumber("Left Target", leftLeader.GetClosedLoopTarget(Constant::pidChannel));
//
//		frc::SmartDashboard::PutNumber("Right Enc Pos", rightLeader.GetSelectedSensorPosition(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Error", rightLeader.GetClosedLoopError(Constant::pidChannel));
//		frc::SmartDashboard::PutNumber("Right Target", rightLeader.GetClosedLoopTarget(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Angle", gyro.GetYaw());
		frc::SmartDashboard::PutString("Waiting to Zero", (waiting) ? "true" : "false");

		switch (autonState) {
			case START:
				frc::SmartDashboard::PutString("Auton State", "Start");
				break;
			case TRAVERSE:
				frc::SmartDashboard::PutString("Auton State", "Traverse");
				break;
			case DROP:
				frc::SmartDashboard::PutString("Auton State", "Drop");
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

		frc::SmartDashboard::PutString("PIDIsOnTarget", (pidController.OnTarget()) ? "true" : "false");
		frc::SmartDashboard::PutNumber("PIDTarget", pidController.GetSetpoint());
		frc::SmartDashboard::PutString("IsEnabled", (pidController.IsEnabled()) ? "true" : "false");
		frc::SmartDashboard::PutString("Turn Degrees Function", inTurnDegrees ? "true" : "false");
		frc::SmartDashboard::PutNumber("PIDError", pidController.GetError());

//		frc::SmartDashboard::PutNumber("P Gain", pidController.GetP());
//		frc::SmartDashboard::PutNumber("I Gain", pidController.GetI());
//		frc::SmartDashboard::PutNumber("D Gain", pidController.GetD());

	}

	bool AutonPositionDeadband(double value, int target) {
		if (fabs(target - value) < Constant::autonPositionDeadbandVal)
			return true;
		else
			return false;
	}

	double TankDriveDeadband(double value) {
		if (value <= Constant::tankDriveDeadbandVal && value >= -Constant::tankDriveDeadbandVal)
			return 0;
		else
			return value;
	}

	double ArcadeDriveDeadband(double value) {
		if (value <= Constant::arcadeDriveDeadbandVal && value >= -Constant::arcadeDriveDeadbandVal)
			return 0;
		else
			return value;
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

	TalonSRX leftLeader { Constant::LeftLeaderID };
	TalonSRX leftFollower { Constant::LeftFollowerID };
	TalonSRX rightLeader { Constant::RightLeaderID };
	TalonSRX rightFollower { Constant::RightFollowerID };
	Joystick joystick1 { 0 };		    // Arcade and Left Tank
	Joystick joystick2 { 1 };			// Right Tank
	PIDMotorOutput pidMotorOutput { &leftLeader, &rightLeader };
	PIDGyroSource pidGyroSource { &gyro };
	PIDController pidController { .025, 0, 0.015, &pidGyroSource, &pidMotorOutput, 0.02 };

	double joyX;
	double joyY;
	double joyZ;
	double leftjoyY;
	double rightjoyY;
	double leftTarget;
	double rightTarget;
	double targetEncPos;
	bool waiting;
	bool inTurnDegrees = false;
	unsigned int traverseStep = 0;
	AHRS gyro { I2C::Port::kMXP };

	std::string colorSides;
};

START_ROBOT_CLASS(Robot)
