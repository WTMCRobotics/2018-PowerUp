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

class Robot: public frc::TimedRobot {
public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		leftLeader.SetSelectedSensorPosition(0, pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, pidChannel, 0);



		gyro.Reset();
		gyro.ZeroYaw();
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

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {
		//UpdateJoystick();
		//Drive();
		UpdateDashboard();
	}

	void TestPeriodic() {
	}

	void UpdateJoystick() {
		leftJoyY = leftJoystick.GetY();
		rightJoyY = rightJoystick.GetY();
	}

	void Drive() {
		// 1500 RPM * 4096 units/rev (resolution * 4) / 600 100ms/min in either direction: velocity control is units/100ms
		//Left motor move
		leftTargetSpeed = Deadband(leftJoyY) * 1500.0 * 4096 / 600;
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, leftTargetSpeed);

		//Right motor move
		rightTargetSpeed = Deadband(leftJoyY) * 1500.0 * 4096 / 600;
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, leftTargetSpeed);
	}

	void SetupMotor() {
		//Left motor setup
		leftLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		leftLeader.SetSensorPhase(true);
		leftLeader.ConfigNominalOutputForward(0, 0);
		leftLeader.ConfigNominalOutputReverse(0, 0);
		leftLeader.ConfigPeakOutputForward(1, 0);
		leftLeader.ConfigPeakOutputReverse(-1, 0);
		leftLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, 1);

		//Right motor setup
		rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, pidChannel, 0);
		rightLeader.SetSensorPhase(true);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, 4);
	}

	void UpdateDashboard() {
		frc::SmartDashboard::PutNumber("Left Encoder Position", leftLeader.GetSelectedSensorPosition(pidChannel));
		frc::SmartDashboard::PutNumber("Left Error", leftLeader.GetClosedLoopError(pidChannel));
		frc::SmartDashboard::PutNumber("Left Target", leftLeader.GetClosedLoopTarget(pidChannel));

		frc::SmartDashboard::PutNumber("Right Encoder Position", rightLeader.GetSelectedSensorPosition(pidChannel));
		frc::SmartDashboard::PutNumber("Right Error", rightLeader.GetClosedLoopError(pidChannel));
		frc::SmartDashboard::PutNumber("Right Target", rightLeader.GetClosedLoopTarget(pidChannel));
		frc::SmartDashboard::PutNumber("Angle", getGyro());
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

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	TalonSRX leftLeader { 1 };
	TalonSRX leftFollower { 2 };
	TalonSRX rightLeader { 4 };
	TalonSRX rightFollower { 3 };
	Joystick rightJoystick { 0 };
	Joystick leftJoystick { 1 };
	double leftJoyY;
	double rightJoyY;
	double leftTargetSpeed;
	double rightTargetSpeed;
	int pidChannel = 0;
	AHRS gyro { SerialPort::kMXP };

	std::string colorSides;
};

START_ROBOT_CLASS(Robot)
