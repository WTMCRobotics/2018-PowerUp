/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <cmath>

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

enum driveMode {
	ARCADE, TANK
};
enum autonStates {
	START, DRIVE_COMMAND, WAIT
} autonState;

class Robot: public frc::TimedRobot {
public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(autoForwardTest, autoForwardTest);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		gyro.Reset();
		gyro.ZeroYaw();
		updateDashboard();

		autonState = START;
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		autonState = START;
		colorSides = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		std::cout << colorSides[0];
		if (colorSides[0] == 'L') {

		} else {

		}

		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == autoForwardTest) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		gyro.Reset();
		gyro.ZeroYaw();
		pidController.SetInputRange(-180, 180);
		pidController.SetOutputRange(-0.5, 0.5);
		pidController.SetContinuous(true);
		pidController.SetPercentTolerance(0.01);
		//pidController.SetPID(0.1, 0.0, 0.0);
		if(!pidController.IsEnabled()) {
			pidController.Enable();
		}
		updateDashboard();
	}

	void AutonomousPeriodic() {
//		if (m_autoSelected == autoForwardTest) {
			switch (autonState) {
			case START:
				leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
				rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
				autonState = DRIVE_COMMAND;
				gyro.ZeroYaw();
				Wait(0.005);
				break;
			case DRIVE_COMMAND:
//				isHere = true;
				pidController.SetSetpoint(90);
				updateDashboard();
				if(pidController.OnTarget()) {
					pidController.Disable();
				}
//				pidController.Disable();
				//driveDistance(36);
				break;
			case WAIT:
				break;
			}
			updateDashboard();
//		} else {
//			// Default Auto goes here
//		}
	}

	void TeleopInit() {

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		pidController.Disable();

		gyro.Reset();
		gyro.ZeroYaw();

		updateDashboard();
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
			if (DeadbandArcade(joyZ) != 0 && Deadband(joyY) == 0 && Deadband(joyX) == 0 ) {
				if (joyZ > 0) {
					leftTarget = -joyZ;
					rightTarget = joyZ;
				} else {
					leftTarget = -joyZ;
					rightTarget = joyZ;
				}
			}
			else if (DeadbandArcade(joyY) <= 0) {
				leftTarget = Deadband(joyY + joyX);
				rightTarget = Deadband(joyY - joyX);
			}

			else {
				leftTarget = Deadband(joyY - joyX);
				rightTarget = Deadband(joyY + joyX);
			}
		} else {
			UpdateJoystickTank();
			leftTarget = Deadband(leftjoyY);
			rightTarget = Deadband(rightjoyY);
		}


		if(leftTarget < 0) {
			leftTarget *= leftTarget;
			leftTarget = -leftTarget;
		}
		else {
			leftTarget *= leftTarget;
		}

		if(rightTarget < 0) {
			rightTarget *= rightTarget;
			rightTarget = -rightTarget;
		}
		else {
			rightTarget *= rightTarget;
		}
		//Left motor move, negative value = forward
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftTarget);

		//Right motor move
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightTarget);
	}

	void driveDistance(double inches) {
		updateDashboard();
		// inches / circumference = number of rotations
		// * pulsesPerRotationQuad = number of pulses in one rotation
		// targetEncPos = position encoder should read
		targetEncPos = (inches * 2 / Constant::circumference) * Constant::pulsesPerRotationQuad;
		// Forward = positive encoder position for left
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetEncPos);
		// Forward = negative encoder position for right
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position, -targetEncPos);

		if(leftLeader.GetSelectedSensorPosition(Constant::pidChannel) == targetEncPos) {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		}
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

		// Set Left PID
		// Values were tested using Web Interface
		leftLeader.Config_kP(Constant::pidChannel, .49, 0);
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
		rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		rightLeader.SetSensorPhase(true);
		rightLeader.SetInverted(false);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, 4);
		rightFollower.SetSensorPhase(true);
		rightFollower.SetInverted(false);
	}

	void updateDashboard() {
		frc::SmartDashboard::PutNumber("Left Enc Pos", leftLeader.GetSelectedSensorPosition(Constant::Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Error", leftLeader.GetClosedLoopError(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Target", leftLeader.GetClosedLoopTarget(Constant::pidChannel));

		frc::SmartDashboard::PutNumber("Right Enc Pos", rightLeader.GetSelectedSensorPosition(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Error", rightLeader.GetClosedLoopError(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Target", rightLeader.GetClosedLoopTarget(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Angle", getGyro());

		frc::SmartDashboard::PutString("Start", (autonState == START) ? "true" : "false");
		frc::SmartDashboard::PutString("Drive", (autonState == DRIVE_COMMAND) ? "true" : "false");
		frc::SmartDashboard::PutString("Wait", (autonState == WAIT) ? "true" : "false");

		frc::SmartDashboard::PutString("PIDIsOnTarget", (pidController.OnTarget()) ? "true" : "false");
		frc::SmartDashboard::PutNumber("PIDTarget", pidController.GetSetpoint());
		frc::SmartDashboard::PutString("IsHere", (isHere) ? "true" : "false");
		frc::SmartDashboard::PutNumber("PIDError", pidController.GetError());



	}

	double getGyro() {
		return gyro.GetYaw();
	}

	bool turnDegrees(bool left, double angle) {
		std::cout<<"turnDegrees" << std::endl;

		currentAngle = getGyro();

		if(currentAngle > angle - 2 && currentAngle < angle + 2) {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			return true;
		}

//		if((currentAngle >= angle && left == false) || (currentAngle <= angle && left == true)) {
//			if (left) {
//				turnDegreesAgain(false, angle);
//			} else {
//				turnDegreesAgain(true, angle);
//			}
//		}

		if (left) {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -(getOutput(angle, currentAngle)));
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -(getOutput(angle, currentAngle)));
			return false;
		}
		else {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  getOutput(angle, currentAngle));
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  getOutput(angle, currentAngle));
			return false;
		}

	}

	void turnDegreesAgain(bool left, double angle) {
//		if((currentAngle >= angle && left == false) || (currentAngle <= angle && left == true)) {
//			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
//			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
//
//			return;
//		}
		std::cout << "TurnAgain" << std::endl;
		currentAngle = getGyro();

		if (left) {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -(getOutput(angle, currentAngle)));
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -(getOutput(angle, currentAngle)));
			return;
		}
		else {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  getOutput(angle, currentAngle));
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  getOutput(angle, currentAngle));
			return;
		}
	}

	double getOutput(double target, double current) {
		if((fabs(target) - fabs(current)) > 15) {
			return 0.3;
		} else {
			return 0.25;
		}

		//		if (pow((target-current)/90, 4.0) >= 0.25) {
//			return pow((target-current)/90, 4.0);
//		} else if ((pow((target-current)/90, 2.5)) <= 0.03) {
//			return 0;
//		} else {
//			return 0.25;
//		}
	}

	double Deadband(double value) {
		if (value <= .15 && value >= -.15)
			return 0;
		else
			return value;
	}

	double DeadbandArcade(double value) {
		if (value <= .2 && value >= -.2)
			return 0;
		else
			return value;
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string autoForwardTest = "Forward Test";
	std::string m_autoSelected;
	TalonSRX leftLeader { Constant::LeftLeaderID };
	TalonSRX leftFollower { Constant::LeftFollowerID };
	TalonSRX rightLeader { Constant::RightLeaderID };
	TalonSRX rightFollower { Constant::RightFollowerID };
	Joystick joystick1 { 0 };		    // Arcade and Left Tank
	Joystick joystick2 { 1 };			// Right Tank
	PIDMotorOutput pidMotorOutput {&leftLeader, &rightLeader};
	PIDGyroSource pidGyroSource {&gyro};
	PIDController pidController {0.025, 0.002, 0.0, &pidGyroSource, &pidMotorOutput};

	double joyX;
	double joyY;
	double joyZ;
	double leftjoyY;
	double rightjoyY;
	double leftTarget;
	double rightTarget;
	double targetEncPos;
	double currentAngle;
	bool turned;
	bool isHere = false;
	AHRS gyro { SerialPort::kMXP };

	std::string colorSides;
};

START_ROBOT_CLASS(Robot)
