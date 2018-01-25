/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <AHRS.h>
#include "Constant.h"

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
		UpdateDashboard();

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
		UpdateDashboard();
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == autoForwardTest) {
			switch (autonState) {
			case START:
				leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel,
						0);
				rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel,
						0);
				autonState = DRIVE_COMMAND;
				break;
			case DRIVE_COMMAND:
				DriveDistance(36);
				autonState = WAIT;
				break;
			case WAIT:
				break;
			}
			UpdateDashboard();
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);

		gyro.Reset();
		gyro.ZeroYaw();

		UpdateDashboard();
	}

	void TeleopPeriodic() {
		Drive(driveMode::ARCADE);
		//Drive(driveMode::TANK);
		UpdateDashboard();
	}

	void TestPeriodic() {
	}

	void UpdateJoystickArcade() {
		JoyY = joystick1.GetY();
		JoyX = -joystick1.GetX();
		JoyZ = joystick1.GetZ();

	}

	void UpdateJoystickTank() {

		leftJoyY = joystick1.GetY();
		rightJoyY = joystick2.GetY();
	}

	void Drive(driveMode mode) {

		if (mode == driveMode::ARCADE) {

			UpdateJoystickArcade();
			if (DeadbandArcade(JoyZ) != 0 && Deadband(JoyY) == 0 && Deadband(JoyX) == 0 ) {
				if (JoyZ > 0) {
					leftTarget = -JoyZ;
					rightTarget = JoyZ;
				} else {
					leftTarget = -JoyZ;
					rightTarget = JoyZ;
				}
			}
			else if (DeadbandArcade(JoyY) <= 0) {
				leftTarget = Deadband(JoyY + JoyX);
				rightTarget = Deadband(JoyY - JoyX);
			}

			else {
				leftTarget = Deadband(JoyY - JoyX);
				rightTarget = Deadband(JoyY + JoyX);
			}
		} else {
			UpdateJoystickTank();
			leftTarget = Deadband(leftJoyY);
			rightTarget = Deadband(rightJoyY);
		}

		//Left motor move, negative value = forward
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
				-leftTarget);

		//Right motor move
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
				rightTarget);
	}

	void DriveDistance(double inches) {
		// inches / circumference = number of rotations
		// * pulsesPerRotationQuad = number of pulses in one rotation
		// targetEncPos = position encoder should read
		targetEncPos = (inches / Constant::circumference)
				* Constant::pulsesPerRotationQuad;
		// Forward = positive encoder position for left
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
				targetEncPos);
		// Forward = negative encoder position for right
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
				-targetEncPos);
	}

	void SetupMotor() {
		//Left motor setup
		leftLeader.ConfigSelectedFeedbackSensor(
				ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		leftLeader.SetSensorPhase(true);
		leftLeader.ConfigNominalOutputForward(0, 0);
		leftLeader.ConfigNominalOutputReverse(0, 0);
		leftLeader.ConfigPeakOutputForward(1, 0);
		leftLeader.ConfigPeakOutputReverse(-1, 0);
		leftLeader.SetNeutralMode(
				ctre::phoenix::motorcontrol::NeutralMode::Brake);

		// Set Left PID
		// P = (percent output * max motor output) / error
		//		50% output when error is 1 rotation away (pulsesPerRotationQuad = encoder counts for 1 rotation)
		//		1023 = max motor output (units for motor output are a scalar from -1023 to +1023)
		leftLeader.Config_kP(Constant::pidChannel,
				(0.5 * 1023) / Constant::pulsesPerRotationQuad, 0);
		rightLeader.Config_kP(Constant::pidChannel,
				(0.5 * 1023) / Constant::pulsesPerRotationQuad, 0);

		leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, 1);

		//Right motor setup
		rightLeader.ConfigSelectedFeedbackSensor(
				ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder,
				Constant::pidChannel, 0);
		rightLeader.SetSensorPhase(true);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(
				ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower,
				4);
	}

	void UpdateDashboard() {
		frc::SmartDashboard::PutNumber("Left Enc Pos",
				leftLeader.GetSelectedSensorPosition(
						Constant::Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Error",
				leftLeader.GetClosedLoopError(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Left Target",
				leftLeader.GetClosedLoopTarget(Constant::pidChannel));

		frc::SmartDashboard::PutNumber("Right Enc Pos",
				rightLeader.GetSelectedSensorPosition(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Error",
				rightLeader.GetClosedLoopError(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Right Target",
				rightLeader.GetClosedLoopTarget(Constant::pidChannel));
		frc::SmartDashboard::PutNumber("Angle", getGyro());

		frc::SmartDashboard::PutNumber("Left Target", leftTarget);
		frc::SmartDashboard::PutNumber("Right Target", rightTarget);
	}

	double getGyro() {
		return gyro.GetYaw();
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
	Joystick joystick1 { 0 };		// Arcade and Left Tank
	Joystick joystick2 { 1 };			// Right Tank
	double JoyX;
	double JoyY;
	double JoyZ;
	double leftJoyY;
	double rightJoyY;
	double leftTarget;
	double rightTarget;
	double targetEncPos;

	AHRS gyro { SerialPort::kMXP };

	std::string colorSides;
};

START_ROBOT_CLASS(Robot)
