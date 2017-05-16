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
		// Initialize objects for hardware
		frc::PowerDistributionPanel	m_PDP {0};
		frc::XboxController	m_controller{0};
		frc::Joystick logitechController{1};
		frc::ADXRS450_Gyro 	m_gyro{frc::SPI::kOnboardCS0};
		frc::Compressor compressor{PCM_ID};

		// Initialize variables for timer
		Timer timer;
		double secondsPassed = 0;
		bool timerOverride = false;

		// Initialize variables for autonomous chooser on dashboard
		frc::SendableChooser<std::string> autoChooser;
		std::string autoLeft = "Left";
		std::string autoMiddle = "Middle";
		std::string autoRight = "Right";
		std::string autoDefault = "Default";
		std::string autoSelected = "";

		// Initialize variables for alliance color and starting location
		DriverStation::Alliance m_allianceColor;
		int m_allianceLocation;

		// Initialize objects for classes
		CANTalonDriveTrain 	m_driveTrain {&m_controller, &m_gyro};
		Winch 				m_winchMotor {&m_PDP};
		GearLift			m_gearLift   {};

		// Variable that stores if it is the practice robot or competition robot
		DigitalInput 		m_diPracticeRobot {DIO_PRACTICE_ROBOT};

		// Initialize gyro variables
		double 	m_gyroAngle;
		double correctedAngle;

		// Initialize XBox controller variables
		double 	m_leftJoystickY;
		double  m_leftJoystickX;
		double 	m_rightJoystickY;
		bool   	m_bLeftBumper;

		// Initialize Logitech controller variables
		double m_logitechYAxis;
		double m_logitechThrottle;
		bool m_logitechTrigger;
		JoystickButton logitechButtonOverrideLimits{&logitechController, 5};
		bool m_logitechOverrideButton;

		frc::LiveWindow* lw = LiveWindow::GetInstance();

		// Enumerator for Autonomous States
		typedef enum
		{
			autoStart	 = 0,
			autoTraverse = 1,
			autoDropGear = 2,
			autoBackup	 = 3,
			autoDone = 4
		} eAutonomousState;
		std::string m_strAutoState[5] = {"autoStart", "autoTraverse", "autoDropGear", "autoBackup","autoDone"};

		// Enumerator for Traverse States
		typedef enum
		{
			traverseNext = 0,
			traverseMove= 1,
			traverseTurn = 2,
			traverseCorrect = 3,
			traverseDone  = 4
		} eTraverseState;
		std::string m_strTraverseState[5] = {"traverseNext", "traverseMove", "traverseTurn", "traverseCorrect", "traverseDone"};

		// Variables for autonomous and traverse state variables
		eAutonomousState m_autoState = autoDone;
		eTraverseState m_traverseState = traverseDone;
		int m_traverseIndex = 0;

		// Arrays for traverse values in autonomous
		double m_angle[AUTO_MOVE_MAX_SEGMENTS];
		double m_leftSpeed[AUTO_MOVE_MAX_SEGMENTS];
		double m_rightSpeed[AUTO_MOVE_MAX_SEGMENTS];
		double m_distance[AUTO_MOVE_MAX_SEGMENTS];
		double m_distanceAdjust[AUTO_MOVE_MAX_SEGMENTS];

		double m_wheelCircumference = 0.00;

	public:

		void RobotInit()
		{
			// Set and output alliance color and location
			m_allianceColor    = DriverStation::GetInstance().GetAlliance();
			m_allianceLocation = DriverStation::GetInstance().GetLocation();
			std::cout << "Alliance Color   : " << m_allianceColor << std::endl;
			std::cout << "Alliance Location: " << m_allianceLocation << std::endl;

			// Jumper is installed on Practice Robot, which pulls DI low
			// Read if practice robot or competition robot
			g_bPracticeRobot = !m_diPracticeRobot.Get();

			// if statement testing if it is the practice robot
			if (g_bPracticeRobot)
				// Practice robot = use practice robot wheel circumference
				m_wheelCircumference = PRACTICE_ROBOT_WHEEL_CIRCUMFERENCE;
			else
				m_wheelCircumference = COMPETITION_ROBOT_WHEEL_CIRCUMFERENCE;

			// Display output from cameras
			frc::CameraServer::GetInstance()->StartAutomaticCapture("Gear Camera", 0);
			frc::CameraServer::GetInstance()->StartAutomaticCapture("Rear View Camera", 1);

			// Display autonomous chooser on dashboard
			autoChooser.AddDefault(autoDefault, autoDefault);
			autoChooser.AddObject(autoLeft, autoLeft);
			autoChooser.AddObject(autoMiddle, autoMiddle);
			autoChooser.AddObject(autoRight, autoRight);
			frc::SmartDashboard::PutData("Autonomous Position Selector", &autoChooser);

			// Reset timer, encoders, and gyroscope
			timer.Reset();
			m_driveTrain.resetEncoders();
			m_gyro.Reset();

			// Update variables on dashboard
			UpdateDashboard();
		} // END of RobotInit() function


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
			// Moved code to a different function when autonomous was not working during competition
			InitializeAutonomous();
		} // END of AutonomousInit() function

		void InitializeAutonomous()
		{
			// Use brake mode in autonomous
			m_driveTrain.setBrakeMode(true);

			// Read which autonomous mode is selected from the dashboard and output
			autoSelected = autoChooser.GetSelected();
			std::cout << "Reselected: " << autoSelected << std::endl;

			// Set traverse index to 0
			m_traverseIndex = 0;

			// Update variables on dashboard
			UpdateDashboard();

			// for loop zeros all traverse arrays
			for (int i=0; i<AUTO_MOVE_MAX_SEGMENTS; i++)
			{
				m_distance[i] = 0.0;
				m_leftSpeed[i] = 0.0;
				m_rightSpeed[i] = 0.0;
				m_angle[i] = 0.0;
				m_distanceAdjust[i] = 0.0;
			} // END of for loop zeroing array elements

			// if left mode is selected
			if(autoSelected == autoLeft)
			{
				// Distance adjustments not used
				m_distanceAdjust[0] = 0;
				m_distanceAdjust[1] = 0;

				// Distance = revolutions
				// 75 inches / wheel circumference = revolutions to do
				// Move 75 inches
				m_distance[0] 	= 75 / m_wheelCircumference;
				// Set left and right speed
				m_leftSpeed[0]	= .5;
				m_rightSpeed[0]	= .5;
				// Turn 60 degrees
				m_angle[0] 		= 60;

				// Move 35 inches
				m_distance[1] 	= 35 / m_wheelCircumference;
				// Set left and right speed
				m_leftSpeed[1]	= .4;
				m_rightSpeed[1]	= .4;
				// Turn 0 degrees
				m_angle[1] 		= 0;
			} // END of if left mode

			// if middle mode is selected
			else if(autoSelected == autoMiddle)
			{
				// Adjust distance by 1 inch per foot
				m_distanceAdjust[0] = kStart2Dist_0 / 12;

				// Move kStart2Dist_0 - m_distanceAdjust inches
				m_distance[0] 	= (kStart2Dist_0 - m_distanceAdjust[0]) / m_wheelCircumference;
				// Set left and right speed
				m_leftSpeed[0]	= kStart2SpeedLf_0;
				m_rightSpeed[0]	= kStart2SpeedRt_0;
				// Set turn angle
				m_angle[0] 		= kStart2Angle_0;
			} // END of if middle mode

			// if right mode is selected
			else if(autoSelected == autoRight)
			{
				// Distance adjustments not used
				m_distanceAdjust[0] = 0;
				m_distanceAdjust[1] = 0;

				// Move 75 inches
				m_distance[0] 	= 75 / m_wheelCircumference;
				// Set left and right speed
				m_leftSpeed[0]	= .5;
				m_rightSpeed[0]	= .5;
				// Turn -60 degrees
				m_angle[0] 		= -60;

				// Move 35 inches
				m_distance[1] 	= 35 / m_wheelCircumference;
				// Set left and right speed
				m_leftSpeed[1]	= .4;
				m_rightSpeed[1]	= .4;
				// Turn 0 degrees
				m_angle[1] 		= 0;
			} // END of right mode

			// if default is selected
			else
			{
				// Adjust distance by 1 inch per foot
				m_distanceAdjust[0] = kStart2Dist_0 / 12;

				// Move kStart2Dist_0 - m_distanceAdjust inches
				m_distance[0] 	= (kStart2Dist_0 - m_distanceAdjust[0]) / m_wheelCircumference;
				// Set left and right speed
				m_leftSpeed[0]	= kStart2SpeedLf_0;
				m_rightSpeed[0]	= kStart2SpeedRt_0;
				// Set turn angle
				m_angle[0] 		= kStart2Angle_0;
			} // END of default mode

			// Set autonomous and traverse state to beginning
			m_autoState = autoStart;
			m_traverseState = traverseNext;

			// Reset the timerOverride, timer, gyro, and encoders
			timerOverride = false;
			timer.Reset();
			m_gyro.Reset();
			m_driveTrain.resetEncoders();
		} // END of InitializeAutonomous() function


		//=================================================================================
		// Autonomous Periodic
		//
		//	Step through Autonomous State Machine
		//
		void AutonomousPeriodic()
		{
			// Periodically call UpdateCompressor() function
			UpdateCompressor();

			// switch statement for autonomous state machine
			switch (m_autoState)
			{
				// if autoStart
				case autoStart:
					// Close lift mechanism
					m_gearLift.Update(0.0, false, false);
					// Continue to autoTraverse
					m_autoState = autoTraverse;
					break;
				// if autoTraverse
				case autoTraverse:
					// AutoTraverse() returns true after a segment completes and
					// there are no more segments
					if (AutoTraverse())
					{
						// if default mode, don't drop the gear
						// Go straight to autoDone
						if(autoSelected == autoDefault)
							m_autoState = autoDone;
						// if NOT default mode, continue to autoDropGear
						else
							m_autoState = autoDropGear;
					} // END of if AutoTraverse() returns true
					break;
				// if autoDropGear
				case autoDropGear:
					// Open lift mechanism
					m_gearLift.Update(0.0, true, false);
					// Stop timer and reset encoders and gyro
					timer.Stop();
					m_driveTrain.resetEncoders();
					m_gyro.Reset();
					// Wait for gyro and encoders to reset
					Wait(.25);
					// Continue to autoBackup
					m_autoState = autoBackup;
					break;
				// if autoBackup
				case autoBackup:
					// AutoMove() returns true when the robot has finished going the desired distance
					if(m_driveTrain.AutoMove(40/m_wheelCircumference, -.4, -.4))
						// After finishing backing up, continue to autoDone
						m_autoState = autoDone;
					break;
				// if autoDone
				case autoDone:
					// Close lift mechanism
					m_gearLift.Update(0.0, false, false);
					break;
			} // END of switch statement for autonomous state machine

			// Update variables on dashboard
			UpdateDashboard();
		} // END of AutonomousPeriodic() function

		bool AutoTraverse(void)
		{
			// if moving to the next segment (new turn and move values)
			if (m_traverseState == traverseNext)
			{
				// if reached max number of segments or segment specifies a speed of 0
				if (m_traverseIndex >= AUTO_MOVE_MAX_SEGMENTS || m_leftSpeed[m_traverseIndex] == 0)
				{
					// Done moving. Return true, so m_autoState can move on to dropping gear
					m_traverseState = traverseDone;
					return true;
				} // END of if max segments or speed 0
				// if max number of segments not reached and the segment doesn't have a speed of 0
				else
				{
					// Continue to traverseMove
					m_traverseState = traverseMove;
					// Reset encoders, gyro, timer
					m_driveTrain.resetEncoders();
					m_gyro.Reset();
					timer.Reset();
					// Wait for gyro and encoders to reset
					Wait(.25);
					// Start timer and update variables on dashboard
					timer.Start();
					UpdateDashboard();
				} // END of if not max segments and speed not 0
			} // END of traverseNext

			// if in the moving part of the segment
			if (m_traverseState == traverseMove)
			{
				// Read the timer
				secondsPassed = timer.Get();

				// if it has been more than 5 seconds and the mode is middle or default
				if(secondsPassed > 5 && (autoSelected == autoMiddle || autoSelected == autoDefault))
				{
					// Stop the robot
					m_driveTrain.Stop();
					// Set timerOverride to true
					timerOverride = true;
					// Continue to traverseTurn
					m_traverseState = traverseTurn;
					// Reset gyro
					m_gyro.Reset();
					// Calculate the turn
					m_driveTrain.AutoCalculateTurn(m_angle[m_traverseIndex], kTurnSpeed);
					// Update variables on dashboard
					UpdateDashboard();
				} // END of if past 5 seconds and mode middle or default
				// if it has been more than 3 seconds and the mode is left or right and it is the 2nd segment
				else if(secondsPassed > 3 && (autoSelected == autoLeft || autoSelected == autoRight) && m_traverseIndex == 1)
				{
					// Stop the robot
					m_driveTrain.Stop();
					// Set timerOverride to true
					timerOverride = true;
					// Continue to traverseTurn
					m_traverseState = traverseTurn;
					// Reset gyro
					m_gyro.Reset();
					// Calculate the turn
					m_driveTrain.AutoCalculateTurn(m_angle[m_traverseIndex], kTurnSpeed);
					// Update variables on dashboard
					UpdateDashboard();
				} // END of if past 3 seconds and mode left or right and 2nd segment
				// if AutoMove() returns true because the robot has finished going the desired distance
				else if (m_driveTrain.AutoMove(m_distance[m_traverseIndex], m_leftSpeed[m_traverseIndex], m_rightSpeed[m_traverseIndex]))
				{
					// Stop the robot
					m_driveTrain.Stop();
					// Continue to traverseTurn
					m_traverseState = traverseTurn;
					// Stop and reset the timer
					timer.Stop();
					timer.Reset();
					// Reset the gyro
					m_gyro.Reset();
					// Calculate the turn
					m_driveTrain.AutoCalculateTurn(m_angle[m_traverseIndex], kTurnSpeed);
					// Update variables on dashboard
					UpdateDashboard();
				} // END of AutoMove() returns true
			} // END of traverseMove

			// if in the turning part of a segment
			if (m_traverseState == traverseTurn)
			{
				// Read values from controllers
				UpdateControlData();
				// Update variables on dashboard
				UpdateDashboard();

				// if the turn doesn't equal 0
				if(m_angle[m_traverseIndex] != 0)
				{
					// if AutoTurn() returns true because the robot has finished turning to the desired angle
					if (m_driveTrain.AutoTurn(m_angle[m_traverseIndex]))
					{
						// if left or right mode
						if(autoSelected == autoLeft || autoSelected == autoRight)
						{
							// Continue to traverseCorrect
							m_traverseState = traverseCorrect;
							// Read values from controllers
							UpdateControlData();
							// Update variables on dashboard
							UpdateDashboard();
							// Wait after turning
							Wait(.25);
						} // END of mode left or right
						// if not left or right mode
						else
						{
							// Reset encoders
							m_driveTrain.resetEncoders();
							// Continue to traverseNext (next segment)
							m_traverseState = traverseNext;
							// Increment the index
							m_traverseIndex++;
						} // END of if mode not left or right
					} // END of if AutoTurn() returns true
				} // END of if turn doesn't equal 0
				// if turn equals 0
				else
				{
					// Reset encoders
					m_driveTrain.resetEncoders();
					// Continue to traverseNext (next segment)
					m_traverseState = traverseNext;
					// Increment the index
					m_traverseIndex++;
				} // END of if turn equals 0
			} // END of traverseTurn

			// if traverseCorrect
			if(m_traverseState == traverseCorrect)
			{
				// Read values from controllers
				UpdateControlData();
				// Update variables on dashboard
				UpdateDashboard();
				// if AutoTurnCorrect() returns true because the robot has turned to the correct angle
				if(m_driveTrain.AutoTurnCorrect(m_angle[m_traverseIndex]))
				{
					// Read the corrected gyro angle and output
					correctedAngle = m_gyro.GetAngle();
					frc::SmartDashboard::PutNumber("Corrected Angle: ", round(correctedAngle, 2));

					// Read values from controllers
					UpdateControlData();
					// Update variables on dashboard
					UpdateDashboard();
					// Continue to traverseNext (next segment)
					m_traverseState = traverseNext;
					// Increment the index
					m_traverseIndex++;
				} // END of if AutoTrunCorrect() returns true
			} // END of traverseCorrect

			// if return true did not trigger because robot in the middle of a segment, return false
			return false;
		} // END of AutoTraverse() function

		void TeleopInit()
		{
			// Reset gyro and stop robot
			m_gyro.Reset();
			m_driveTrain.Stop();

			// Output to dashboard alliance color and location
			frc::SmartDashboard::PutString("Alliance Color    : ", (m_allianceColor == DriverStation::Alliance::kRed) ? "Red" : "Blue");
			frc::SmartDashboard::PutNumber("Alliance Location : ", m_allianceLocation);

			// Set to coast mode
			m_driveTrain.setBrakeMode(false);
		} // END of TeleopInit() function

		void UpdateControlData()
		{
			// Read current gyro angle
			m_gyroAngle = m_gyro.GetAngle();

			// Read x and y values of left stick
			m_leftJoystickY  = m_controller.GetY(frc::GenericHID::kLeftHand);
			m_leftJoystickX = m_controller.GetX(frc::GenericHID::kLeftHand);
			// Read y value of right stick
			m_rightJoystickY = m_controller.GetY(frc::GenericHID::kRightHand);

			// Read bumper value on the left side
			m_bLeftBumper = m_controller.GetBumper(frc::GenericHID::kLeftHand);

			// Read y value of Logitech stick
			m_logitechYAxis = logitechController.GetY();
			// Read trigger value of Logitech stick
			m_logitechTrigger = logitechController.GetTrigger();
			// Read throttle value of Logitech stick
			m_logitechThrottle = logitechController.GetThrottle();

			// Read value of the override button
			m_logitechOverrideButton = logitechButtonOverrideLimits.Get();
		} // END of UpdateControlData() function

		void TeleopPeriodic()
		{
			// Read values from controller
			UpdateControlData();
			// Use Update() from drivetrain to move the robot with joystick
			m_driveTrain.Update(m_leftJoystickY, m_rightJoystickY, m_bLeftBumper);

			// Use Update() from gear lift to move the gear lift up/down and open/close
			m_gearLift.Update(m_logitechYAxis, m_logitechTrigger, m_logitechOverrideButton);
			// Use Update() from winch to move the winch
			m_winchMotor.Update(m_logitechThrottle);

			// Periodically call UpdateCompressor()
			UpdateCompressor();

			// Update variables on dashboard
			UpdateDashboard();
		} // END of TeleopPeriodic() function

		void TestPeriodic()
		{
			//lw->Run();
		} // END of TestPeriodic() function

		void UpdateCompressor(void)
		{
			// if not enough pressure
			if(!compressor.GetPressureSwitchValue())
				// Start compressor
				compressor.Start();
			// if enough pressure
			else
				// Stop compressor
				compressor.Stop();
		} // END of UpdateCompressor() function

		void UpdateDashboard(void)
		{
			// Update drivetrain variables
			m_driveTrain.UpdateStats();
			// Update drivetrain variables on dashboard
			m_driveTrain.DriveTrainUpdateDashboard();

			// Read values from controllers
			UpdateControlData();

			// Put practice robot or competition robot on dashboard
			frc::SmartDashboard::PutString("Robot : ", (g_bPracticeRobot) ? "Practice Robot" : "Competition Robot");

			// Put autonomous variables on dashboard
			frc::SmartDashboard::PutString("AutoState     : ", m_strAutoState[m_autoState]);
			frc::SmartDashboard::PutString("TraverseState : ", m_strTraverseState[m_traverseState]);
			frc::SmartDashboard::PutString("Auto Selected : ", autoSelected);
			frc::SmartDashboard::PutNumber("Auto Distance     : ", round(m_distance[m_traverseIndex], 2));
			frc::SmartDashboard::PutNumber("Auto Angle    : ", round(m_angle[m_traverseIndex], 2));
			frc::SmartDashboard::PutNumber("Auto Left Speed      : ", round(m_leftSpeed[m_traverseIndex], 2));
			frc::SmartDashboard::PutNumber("Auto Right Speed    : ", round(m_rightSpeed[m_traverseIndex], 2));

			// Put encoder and motor variables on dashboard
			frc::SmartDashboard::PutNumber("Encoder Velocity Difference : ", round(m_driveTrain.GetEncoderVelocityDifference(), 2));
			// Left Side
			frc::SmartDashboard::PutNumber("Left Speed     : ", round(m_driveTrain.GetLeftSpeed(), 2));
			frc::SmartDashboard::PutNumber("Left Position  : ", round(m_driveTrain.GetLeftPosition(), 2));
			frc::SmartDashboard::PutNumber("Left Enc. Pos. : ", round(m_driveTrain.GetLeftEncoderPos(), 2));
			frc::SmartDashboard::PutNumber("Left Enc. Vel. : ", round(m_driveTrain.GetLeftEncoderVel(), 2));
			// Right Side
			frc::SmartDashboard::PutNumber("Right Speed    : ", round(m_driveTrain.GetRightSpeed(), 2));
			frc::SmartDashboard::PutNumber("Right Position : ", round(m_driveTrain.GetRightPosition(), 2));
			frc::SmartDashboard::PutNumber("Right Enc. Pos.: ", round(m_driveTrain.GetRightEncoderPos(), 2));
			frc::SmartDashboard::PutNumber("Right Enc. Vel.: ", round(m_driveTrain.GetRightEncoderVel(), 2));

			// Put XBox controller values and calculated speed commands on dashboard
			frc::SmartDashboard::PutBoolean("Slow Speed :  ", m_bLeftBumper);
			// Left Side
			frc::SmartDashboard::PutNumber("Left Joystick  : ", round(m_leftJoystickY, 2));
			frc::SmartDashboard::PutNumber("Left Command   : ", round(m_driveTrain.GetLeftTarget(), 2));
			// Right Side
			frc::SmartDashboard::PutNumber("Right Joystick : ", round(m_rightJoystickY, 2));
			frc::SmartDashboard::PutNumber("Right Command  : ", round(m_driveTrain.GetRightTarget(), 2));

			// Put gear lift variables on dashboard
			frc::SmartDashboard::PutBoolean("   GearLift Up     ", m_gearLift.IsUp());
			frc::SmartDashboard::PutBoolean("   GearLift Down   ", m_gearLift.IsDown());
			frc::SmartDashboard::PutBoolean("   GearLift Clamp  ", m_gearLift.IsOpen());
			frc::SmartDashboard::PutBoolean("   Limit Override  ", m_logitechOverrideButton);

			// Put winch variable on dashboard
			frc::SmartDashboard::PutNumber("Winch Trigger  : ", round(m_logitechThrottle, 2));

			// Put current gyro angle on dashboard
			frc::SmartDashboard::PutNumber("Gyro Angle     : ", round(m_gyroAngle, 2));

			// Put compressor variables on dashboard
			frc::SmartDashboard::PutBoolean("Switch Valve : ", compressor.GetPressureSwitchValue());
			frc::SmartDashboard::PutBoolean("Compressor On : ", compressor.Enabled());

			// Put time variables on dashboard
			frc::SmartDashboard::PutNumber("Seconds Passed   : ", secondsPassed);
			frc::SmartDashboard::PutBoolean("Timer Override Activated: ", timerOverride);
		} // END of UpdateDashboard() function

		void DisabledInit()
		{
			printf("%s\n", "DisabledInit");
		} // END of DisabledInit() function

		void DisabledPeriodic()
		{
			m_driveTrain.Stop();
		} // END of DisabledPeriod();

		void RobotPeriodic() {}

		double round(double value, int numDecimals)
		{
			// Multiply value, truncate, and divide
			return trunc(value * pow(10, numDecimals)) / pow(10, numDecimals);
		} // END of round() function
};


START_ROBOT_CLASS(Robot)
