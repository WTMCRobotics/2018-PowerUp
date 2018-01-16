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
#include <XboxController.h>
#include <AHRS.h>

class Robot : public frc::TimedRobot {
public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
		if(colorSides[0] == 'L')
		{

		}
		else
		{

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

	void TeleopPeriodic()
	{
		//UpdateJoystick();
		//Drive();
		//UpdateDashboard();
	}

	void TestPeriodic() {}

	void UpdateJoystick()
	{
		leftStick = xboxController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
	}

	void Drive()
	{
		// 1500 RPM * 4096 units/rev (resolution * 4) / 600 100ms/min in either direction: velocity control is units/100ms
		targetSpeed = Deadband(leftStick) * 1500.0 * 4096 / 600;
		motor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, targetSpeed);
	}

	void SetupMotor()
	{
		motor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
		motor.SetSensorPhase(true);
		motor.ConfigNominalOutputForward(0, 0);
		motor.ConfigNominalOutputReverse(0, 0);
		motor.ConfigPeakOutputForward(1, 0);
		motor.ConfigPeakOutputReverse(-1, 0);
		motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	}

	void UpdateDashboard()
	{
		frc::SmartDashboard::PutNumber("Encoder Position", motor.GetSelectedSensorPosition(0));
		frc::SmartDashboard::PutNumber("Error", motor.GetClosedLoopError(0));
		frc::SmartDashboard::PutNumber("Target", motor.GetClosedLoopTarget(0));
		frc::SmartDashboard::PutNumber("Angle", getGyro());
	}

	double getGyro()
	{
		return gyro.GetYaw();
	}

	double Deadband(double value)
	{
		if(value <= .15 && value >= -.15)
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
	TalonSRX motor{1};
	XboxController xboxController{0};
	double leftStick;
	double targetSpeed;
	AHRS gyro{SerialPort::kMXP};

	std::string colorSides;
};

START_ROBOT_CLASS(Robot)
