#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cmath>

#include <Timer.h>

#include <IterativeRobot.h>
#include <DriverStation.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/Sendable.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CameraServer.h>
#include <tables/ITable.h>
#include <Preferences.h>

#include "RobotDefs.h"

#include <PowerDistributionPanel.h>
#include <ADXRS450_Gyro.h>
#include <DigitalInput.h>
#include <Joystick.h>
#include <Buttons/JoystickButton.h>
#include <Compressor.h>

#include "CANTalonDrivetrain.h"
#include "Winch.h"
#include "GearLift.h"

bool g_bPracticeRobot = false;
const double maxSpeed = 2000;

class Robot: public frc::IterativeRobot
{

private:
	frc::PowerDistributionPanel	m_PDP {0};
	frc::XboxController	m_controller{0};
	frc::Joystick logitechController{1};
	frc::ADXRS450_Gyro 	m_gyro{frc::SPI::kOnboardCS0};
	JoystickButton logitechButtonOverrideLimits{&logitechController, 5};
	frc::Compressor compressor{PCM_ID};

	Preferences *preferences;
	double turnCorrectionSpeedModifier;

	Timer timer;
	double secondsPassed = 0;

	frc::SendableChooser<std::string> autoChooser;
	std::string autoLeft = "Left";
	std::string autoMiddle = "Middle";
	std::string autoRight = "Right";
	std::string autoDefault = "Default";
	std::string autoSelected = "";

	bool firstTimeAuto;
	bool timerOverride = false;

	DriverStation::Alliance m_allianceColor;
	int m_allianceLocation;


	CANTalonDriveTrain 	m_driveTrain {&m_controller, &m_gyro};
	Winch 				m_winchMotor {&m_PDP};
	GearLift			m_gearLift   {};
	DigitalInput 		m_diPracticeRobot {DIO_PRACTICE_ROBOT};

	double 	m_gyroAngle;
	double correctedAngle;
	double 	m_leftJoystickY;
	double  m_leftJoystickX;
	double 	m_rightJoystickY;
	double 	m_leftTrigger;
	double 	m_rightTrigger;
	bool   	m_bLeftBumper;
	bool   	m_bRightBumper;
	bool	m_bButtonA;
	bool	m_bButtonB;
	bool	m_bButtonX;
	bool	m_bButtonY;

	double m_logitechYAxis;
	double m_logitechThrottle;
	bool m_logitechTrigger;
	bool m_logitechOverrideButton;

	frc::LiveWindow* lw = LiveWindow::GetInstance();

	// autonomous states
	//
	typedef enum
	{
		autoStart	 = 0,
		autoTraverse = 1,
		autoDropGear = 2,
		autoBackup	 = 3,
		autoDone = 4
	} eAutonomousState;
	std::string m_strAutoState[5] = {"autoStart", "autoTraverse", "autoDropGear", "autoBackup","autoDone"};


	// traverse states
	//
	typedef enum
	{
		traverseNext = 0,
		traverseMove= 1,
		traverseTurn = 2,
		traverseCorrect = 3,
		traverseDone  = 4
	} eTraverseState;
	std::string m_strTraverseState[5] = {"traverseNext", "traverseMove", "traverseTurn", "traverseCorrect", "traverseDone"};


	eAutonomousState m_autoState = autoDone;
	eTraverseState m_traverseState = traverseDone;
	int m_traverseIndex = 0;

	double m_angle[AUTO_MOVE_MAX_SEGMENTS];
	double m_leftSpeed[AUTO_MOVE_MAX_SEGMENTS];
	double m_rightSpeed[AUTO_MOVE_MAX_SEGMENTS];
	double m_distance[AUTO_MOVE_MAX_SEGMENTS];
	double m_distanceAdjust[AUTO_MOVE_MAX_SEGMENTS];

	double m_wheelCircumfrence = 0.00;

public:

	void RobotInit()
	{
		printf("%s\n", "RobotInit");
		std::cout << "RobotInit" << std::endl;
		m_allianceColor    = DriverStation::GetInstance().GetAlliance();
		m_allianceLocation = DriverStation::GetInstance().GetLocation();

		std::cout << "Alliance Color   : " << m_allianceColor << std::endl;
		std::cout << "Alliance Location: " << m_allianceLocation << std::endl;

		// Jumper is installed on Practice Robot, which pulls DI low
		g_bPracticeRobot = !m_diPracticeRobot.Get();

		if (g_bPracticeRobot)
			m_wheelCircumfrence = PRACTICE_ROBOT_WHEEL_CIRCUMFRENCE;
		else
			m_wheelCircumfrence = COMPETITION_ROBOT_WHEEL_CIRCUMFRENCE;

		frc::CameraServer::GetInstance()->StartAutomaticCapture("Gear Camera", 0);
		frc::CameraServer::GetInstance()->StartAutomaticCapture("Rear View Camera", 1);

		firstTimeAuto = true;

		autoChooser.AddDefault(autoDefault, autoDefault);
		autoChooser.AddObject(autoLeft, autoLeft);
		autoChooser.AddObject(autoMiddle, autoMiddle);
		autoChooser.AddObject(autoRight, autoRight);

		frc::SmartDashboard::PutData("Autonomous Position Selector", &autoChooser);

		preferences = Preferences::GetInstance();

		timer.Reset();
		m_driveTrain.resetEncoders();
		m_gyro.Reset();
		UpdateDashboard();
	}


	//=============================================================================
	// Autonomous Initialize
	//
	//	Starting Location key:
	//			+-----------------+
	//		 	| 1				1 |
	//	Blue 	| x 			x | Red
	//	Alliance| 2				2 | Alliance
	//		 	| 3				3 |
	//			+-----------------+
	//
	void AutonomousInit() //override
	{
		printf("%s\n", "AutoInit");
		InitializeAutonomous();
	}


	//=================================================================================
	// Autonomous Periodic
	//
	//	Step through Autonomous State Machine
	//
	void AutonomousPeriodic()
	{

		if(firstTimeAuto)
		{
			printf("%s\n", "AutoPeriodic");
			InitializeAutonomous();
		}
		firstTimeAuto = false;

		UpdateCompressor();
		switch (m_autoState)
		{
			case autoStart:
				m_gearLift.Update(0.0, false, false);
				m_autoState = autoTraverse;
				break;
			case autoTraverse:
				// AutoTraverse() returns true after a segment completes and
				// the either isn't another segment allowed, or the next segment
				// has a speed of 0
				if (AutoTraverse())
				{
					if(autoSelected == autoDefault)
					{
						m_autoState = autoDone;
					}
					else
					{
						m_autoState = autoDropGear;
					}
				}
				break;
			case autoDropGear:
				m_gearLift.Update(0.0, true, false);
				timer.Stop();
				m_driveTrain.resetEncoders();
				m_gyro.Reset();
				Wait(.25);
				m_autoState = autoBackup;
				break;
			case autoBackup:
				// Change -.2 to -.4 for competition
				if(m_driveTrain.AutoMove(40/m_wheelCircumfrence, -.4, -.4))
					m_autoState = autoDone;
				break;
			case autoDone:
				m_gearLift.Update(0.0, false, false);
				break;
		}

		UpdateDashboard();
	}


	bool AutoTraverse(void)
	{
		// if moving to the next segment (new turn and move values)
		if (m_traverseState == traverseNext)
		{
			// if reached max number of segments or segment specifies a speed of 0
			if (m_traverseIndex >= AUTO_MOVE_MAX_SEGMENTS ||
				m_leftSpeed[m_traverseIndex] == 0)
			{
				// Done moving. Return true, so m_autoState can move on to dropping gear
				m_traverseState = traverseDone;
				return true;
			}
			// if max number of segments not reached and the segment doesn't have a speed of 0
			else
			{
				m_traverseState = traverseMove;
				m_driveTrain.resetEncoders();
				m_gyro.Reset();
				timer.Reset();
				timer.Start();
				Wait(.25);
				UpdateDashboard();
			}
		}

		// if in the moving part of the segment
		if (m_traverseState == traverseMove)
		{
			secondsPassed = timer.Get();
			// AutoMoveUpdate() returns true when the robot has moved the correct distance
			if(secondsPassed > 5 && (autoSelected == autoMiddle || autoSelected == autoDefault))
			{
				m_driveTrain.Stop();
				timerOverride = true;
				m_traverseState = traverseTurn;
				m_gyro.Reset();
				m_driveTrain.AutoCalculateTurn(m_angle[m_traverseIndex], kTurnSpeed);
				UpdateDashboard();
			}
			else if(secondsPassed > 3 && (autoSelected == autoLeft || autoSelected == autoRight) && m_traverseIndex == 1)
			{
				m_driveTrain.Stop();
				timerOverride = true;
				m_traverseState = traverseTurn;
				m_gyro.Reset();
				m_driveTrain.AutoCalculateTurn(m_angle[m_traverseIndex], kTurnSpeed);
				UpdateDashboard();
			}
			else if (m_driveTrain.AutoMove(m_distance[m_traverseIndex], m_leftSpeed[m_traverseIndex], m_rightSpeed[m_traverseIndex]))
			{
				m_traverseState = traverseTurn;
				timer.Stop();
				timer.Reset();
				m_gyro.Reset();
				m_driveTrain.AutoCalculateTurn(m_angle[m_traverseIndex], kTurnSpeed);
				UpdateDashboard();
			}
		}

		// if in the turning part of a segment
		if (m_traverseState == traverseTurn)
		{
			UpdateControlData();
			UpdateDashboard();
			// AutoTurnUpdate() returns true when robot has turned the correct angle
			if(m_angle[m_traverseIndex] != 0)
			{
				if (m_driveTrain.AutoTurn(m_angle[m_traverseIndex]))
				{
					if(autoSelected == autoLeft || autoSelected == autoRight)
					{
						m_traverseState = traverseCorrect;
						UpdateControlData();
						UpdateDashboard();
						Wait(.25);
					}
					else
					{
						m_driveTrain.resetEncoders();
						m_traverseState = traverseNext;
						// Increment the index
						m_traverseIndex++;
					}
				}
			}
			else
			{
				m_driveTrain.resetEncoders();
				m_traverseState = traverseNext;
				// Increment the index
				m_traverseIndex++;
			}
		}

		if(m_traverseState == traverseCorrect)
		{
			turnCorrectionSpeedModifier = 1;
			UpdateControlData();
			UpdateDashboard();
			if(m_driveTrain.AutoTurnCorrect(m_angle[m_traverseIndex], turnCorrectionSpeedModifier))
			{
				correctedAngle = m_gyro.GetAngle();
				frc::SmartDashboard::PutNumber("Corrected Angle: ", round(correctedAngle, 2));
				UpdateControlData();
				UpdateDashboard();
				// After turning is done, try to go to the next segment
				m_traverseState = traverseNext;
				// Increment the index
				m_traverseIndex++;
			}
		}

		// if return true did not trigger because robot in the middle of a segment, return false
		return false;
	}

	void TeleopInit()
	{
		firstTimeAuto = true;
		m_gyro.Reset();
		m_driveTrain.Stop();

		frc::SmartDashboard::PutString("Alliance Color    : ", (m_allianceColor == DriverStation::Alliance::kRed) ? "Red" : "Blue");
		frc::SmartDashboard::PutNumber("Alliance Location : ", m_allianceLocation);

		m_driveTrain.setBrakeMode(false);
	}

	void UpdateControlData()
	{
		m_gyroAngle = m_gyro.GetAngle();
		m_leftJoystickY  = m_controller.GetY(frc::GenericHID::kLeftHand);
		m_leftJoystickX = m_controller.GetX(frc::GenericHID::kLeftHand);
		m_rightJoystickY = m_controller.GetY(frc::GenericHID::kRightHand);

		m_leftTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand);
		m_rightTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);

		m_bLeftBumper = m_controller.GetBumper(frc::GenericHID::kLeftHand);
		m_bRightBumper = m_controller.GetBumper(frc::GenericHID::kRightHand);

		m_bButtonA = m_controller.GetAButton();
		m_bButtonB = m_controller.GetBButton();
		m_bButtonX = m_controller.GetXButton();
		m_bButtonY = m_controller.GetYButton();

		m_logitechYAxis = logitechController.GetY();
		m_logitechTrigger = logitechController.GetTrigger();
		m_logitechThrottle = logitechController.GetThrottle();
		m_logitechOverrideButton = logitechButtonOverrideLimits.Get();
	}

	double m_leftPosOffset = 0.0;
	double m_rightPosOffset = 0.0;

	void TeleopPeriodic()
	{
		UpdateControlData();

		/*if(m_bRightBumper)
			m_driveTrain.ArcadeDrive(m_leftJoystickY, m_leftJoystickX, m_bLeftBumper);
		else*/
			m_driveTrain.Update(m_leftJoystickY, m_rightJoystickY, m_bLeftBumper);

		m_gearLift.Update(m_logitechYAxis, m_logitechTrigger, m_logitechOverrideButton);
		m_winchMotor.Update(m_logitechThrottle);
		UpdateCompressor();

		UpdateDashboard();
	}

	void TestPeriodic()
	{
		//lw->Run();
	}


	void UpdateCompressor(void)
	{
		if(!compressor.GetPressureSwitchValue())
			compressor.Start();
		else
			compressor.Stop();
	}

	void UpdateDashboard(void)
	{
		m_driveTrain.UpdateStats();
		m_driveTrain.DriveTrainUpdateDashboard();
		UpdateControlData();

		frc::SmartDashboard::PutString("Robot : ", (g_bPracticeRobot) ? "Practice Robot" : "Competition Robot");
		frc::SmartDashboard::PutString("AutoState     : ", m_strAutoState[m_autoState]);
		frc::SmartDashboard::PutString("TraverseState : ", m_strTraverseState[m_traverseState]);
		frc::SmartDashboard::PutString("Auto Selected : ", autoSelected);

		frc::SmartDashboard::PutNumber("Encoder Velocity Difference : ", round(m_driveTrain.GetEncoderVelocityDifference(), 2));

		frc::SmartDashboard::PutNumber("Left Joystick  : ", round(m_leftJoystickY, 2));
		frc::SmartDashboard::PutNumber("Left Command   : ", round(m_driveTrain.GetLeftTarget(), 2));
		frc::SmartDashboard::PutNumber("Left Speed     : ", round(m_driveTrain.GetLeftSpeed(), 2));
		frc::SmartDashboard::PutNumber("Left Position  : ", round(m_driveTrain.GetLeftPosition(), 2));
		frc::SmartDashboard::PutNumber("Left Enc. Pos. : ", round(m_driveTrain.GetLeftEncoderPos(), 2));
		frc::SmartDashboard::PutNumber("Left Enc. Vel. : ", round(m_driveTrain.GetLeftEncoderVel(), 2));

		frc::SmartDashboard::PutNumber("Right Joystick : ", round(m_rightJoystickY, 2));
		frc::SmartDashboard::PutNumber("Right Command  : ", round(m_driveTrain.GetRightTarget(), 2));
		frc::SmartDashboard::PutNumber("Right Speed    : ", round(m_driveTrain.GetRightSpeed(), 2));
		frc::SmartDashboard::PutNumber("Right Position : ", round(m_driveTrain.GetRightPosition(), 2));
		frc::SmartDashboard::PutNumber("Right Enc. Pos.: ", round(m_driveTrain.GetRightEncoderPos(), 2));
		frc::SmartDashboard::PutNumber("Right Enc. Vel.: ", round(m_driveTrain.GetRightEncoderVel(), 2));

		frc::SmartDashboard::PutBoolean("   GearLift Up     ", m_gearLift.IsUp());
		frc::SmartDashboard::PutBoolean("   GearLift Down   ", m_gearLift.IsDown());
		frc::SmartDashboard::PutBoolean("   GearLift Clamp  ", m_gearLift.IsClamped());
		frc::SmartDashboard::PutNumber("Winch Trigger  : ", round(m_logitechThrottle, 2));

		frc::SmartDashboard::PutNumber("TraverseState  : ", m_traverseState);

		frc::SmartDashboard::PutNumber("Gyro Angle     : ", round(m_gyroAngle, 2));

		/*frc::SmartDashboard::PutNumber("Y-Axis Joystick Test  : ", round(m_logitechYAxis, 2));
		frc::SmartDashboard::PutBoolean("Trigger Button : ", m_logitechTrigger);
		frc::SmartDashboard::PutNumber("Slider Joystick Test : ", round(m_logitechThrottle, 2));*/
		frc::SmartDashboard::PutBoolean("   Limit Override  ", m_logitechOverrideButton);
		frc::SmartDashboard::PutBoolean("Slow Speed :  ", m_bLeftBumper);


		frc::SmartDashboard::PutBoolean("Switch Valve : ", compressor.GetPressureSwitchValue());
		frc::SmartDashboard::PutBoolean("Compressor On : ", compressor.Enabled());

		frc::SmartDashboard::PutBoolean("First Time Auto  : ", firstTimeAuto);

		frc::SmartDashboard::PutNumber("Seconds Passed   : ", secondsPassed);
		frc::SmartDashboard::PutBoolean("Timer Override Activated: ", timerOverride);

		frc::SmartDashboard::PutNumber("Auto Distance     : ", round(m_distance[m_traverseIndex], 2));
		frc::SmartDashboard::PutNumber("Auto Angle    : ", round(m_angle[m_traverseIndex], 2));
		frc::SmartDashboard::PutNumber("Auto Left Speed      : ", round(m_leftSpeed[m_traverseIndex], 2));
		frc::SmartDashboard::PutNumber("Auto Right Speed    : ", round(m_rightSpeed[m_traverseIndex], 2));
	}

	void InitializeAutonomous()
	{
		m_driveTrain.setBrakeMode(true);
		autoSelected = autoChooser.GetSelected();

		std::cout << "Reselected: " << autoSelected << std::endl;


		m_traverseIndex = 0;

		UpdateDashboard();

		// clear all traverse segments
		//
		for (int i=0; i<AUTO_MOVE_MAX_SEGMENTS; i++)
		{
			m_distance[i] = 0.0;
			m_leftSpeed[i] = 0.0;
			m_rightSpeed[i] = 0.0;
			m_angle[i] = 0.0;
			m_distanceAdjust[i] = 0.0;
		}

		if(autoSelected == autoLeft)
		{
			m_distanceAdjust[0] = 0;
			m_distanceAdjust[1] = 0;

			m_angle[0] 		= 60;
			m_distance[0] 	= 75 / m_wheelCircumfrence;
			m_leftSpeed[0]	= .5;
			m_rightSpeed[0]	= .5;

			m_angle[1] 		= 0;
			m_distance[1] 	= 35 / m_wheelCircumfrence;
			m_leftSpeed[1]	= .4;
			m_rightSpeed[1]	= .4;
		}
		else if(autoSelected == autoMiddle)
		{
			m_distanceAdjust[0] = kStart2Dist_0 / 12;

			m_angle[0] 		= kStart2Angle_0;
			m_distance[0] 	= (kStart2Dist_0 - m_distanceAdjust[0]) / m_wheelCircumfrence;
			m_leftSpeed[0]	= kStart2SpeedLf_0;
			m_rightSpeed[0]	= kStart2SpeedRt_0;
		}
		else if(autoSelected == autoRight)
		{
			m_distanceAdjust[0] = 0;
			m_distanceAdjust[1] = 0;

			m_distance[0] 	= 75 / m_wheelCircumfrence;
			m_leftSpeed[0]	= .5;
			m_rightSpeed[0]	= .5;
			m_angle[0] 		= -60;

			m_angle[1] 		= 0;
			m_distance[1] 	= 35 / m_wheelCircumfrence;
			m_leftSpeed[1]	= .4;
			m_rightSpeed[1]	= .4;
		}
		else
		{
			m_distanceAdjust[0] = kStart2Dist_0 / 12;

			m_angle[0] 		= kStart2Angle_0;
			m_distance[0] 	= (kStart2Dist_0 - m_distanceAdjust[0]) / m_wheelCircumfrence;
			m_leftSpeed[0]	= kStart2SpeedLf_0;
			m_rightSpeed[0]	= kStart2SpeedRt_0;
		}

		// Now able to run the state machine in autonomous
		m_autoState = autoStart;
		m_traverseState = traverseNext;


		timerOverride = false;
		timer.Reset();
		m_gyro.Reset();
		m_driveTrain.resetEncoders();
	}

	void DisabledInit()
	{
		printf("%s\n", "DisabledInit");
		firstTimeAuto = true;
	}

	void DisabledPeriodic()
	{
		m_driveTrain.Stop();
	}

	void RobotPeriodic() {}


	double round(double value, int numDecimals)
	{
		return trunc(value * pow(10, numDecimals)) / pow(10, numDecimals);
	}
};


START_ROBOT_CLASS(Robot)
