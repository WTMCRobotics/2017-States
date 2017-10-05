/*
 * CANTalonDriveTrain.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 */

#include <CANTalonDriveTrain.h>
#include <iostream>

CANTalonDriveTrain::CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro*  pGyro)
{
	// Store passed pointers into private pointers in class
	m_pController = pController;
	m_pGyro		  = pGyro;

	// Set speeds to 0
	m_leftSpeed 	= 0.0;
	m_rightSpeed 	= 0.0;

#if defined(MODE_Voltage)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPercentVbus);
	m_rightMasterDrive.Set(0);

#elif defined(MODE_Speed)
	// Setup left side in speed mode
	m_leftMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_leftMasterDrive.Set(0.0);
	m_leftMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_leftMasterDrive.ConfigEncoderCodesPerRev(DRIVE_ENCDR_STEPS);
	m_leftMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_leftMasterDrive.SetVoltageRampRate(DRIVE_RAMP_VoltsPerSec);
	m_leftMasterDrive.SetSensorDirection(true);
	m_leftMasterDrive.SetPID(DRIVE_PID_P_GAIN, DRIVE_PID_I_GAIN, DRIVE_PID_D_GAIN);

	// Setup right side in speed mode
	m_rightMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_rightMasterDrive.Set(0.0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(DRIVE_ENCDR_STEPS);
	m_rightMasterDrive.ConfigNominalOutputVoltage(+0.0f, -0.0f);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMasterDrive.SetVoltageRampRate(DRIVE_RAMP_VoltsPerSec);
	m_rightMasterDrive.SetSensorDirection(true);
	m_rightMasterDrive.SetPID(DRIVE_PID_P_GAIN, DRIVE_PID_I_GAIN, DRIVE_PID_D_GAIN);

#elif defined(MODE_Position)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPosition);
	m_rightMasterDrive.Set(0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
#else
#error No mode selected for TallonSRX
#endif

	// Setup second motor on left side as follower
	m_leftSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_leftSlaveDrive.Set(CAN_ID_LEFTMASTER);

	// Setup second motor on right side as follower
	m_rightSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_rightSlaveDrive.Set(CAN_ID_RIGHTMASTER);

} // END of CANTalonDriveTrain() constructor

CANTalonDriveTrain::~CANTalonDriveTrain() {}

void CANTalonDriveTrain::UpdateStats(void)
{
	// Read both left and right speeds
	m_leftSpeed  = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	// Read both left and right positions
	m_leftPosition  = m_leftMasterDrive.GetPosition();
	m_rightPosition = m_rightMasterDrive.GetPosition();

	// Read both left and right encoder positions
	m_leftEncoderPos  = m_leftMasterDrive.GetEncPosition();
	m_rightEncoderPos = m_rightMasterDrive.GetEncPosition();

	// Read both left and right encoder velocities
	m_leftEncoderVel  = m_leftMasterDrive.GetEncVel();
	m_rightEncoderVel = m_rightMasterDrive.GetEncVel();
} // END of UpdateStats() function

void CANTalonDriveTrain::Stop(void)
{
	// Set left speed variable to 0 and motor to 0
	m_leftSpeed = 0.0;
	m_leftMasterDrive.Set(0);

	// Set right speed variable to 0 and motor to 0
	m_rightSpeed = 0.0;
	m_rightMasterDrive.Set(0);
} // END of Stop() function

void CANTalonDriveTrain::Update(double leftCommand, double rightCommand, bool slowSpeed)
{
	// Calculate target based on joystick controls
	m_leftTarget  = Deadband(leftCommand)  * DRIVE_MAX_SPEED * m_speedFactor;
	m_rightTarget = Deadband(rightCommand) * DRIVE_MAX_SPEED * m_speedFactor;

	// if slow speed button is pressed
	if (slowSpeed)
	{
		// Multiply target to scale down
		m_leftTarget  *= kSlowSpeedFactor;
		m_rightTarget *= kSlowSpeedFactor;
	} // END of if slow speed button pressed

	// Set motors to the respective targets
	m_leftMasterDrive.Set(-m_leftTarget);
	m_rightMasterDrive.Set(m_rightTarget);

	// Update encoder value variables and motor speeds and positions
	UpdateStats();
} // END of Update() function

// Not implemented
void CANTalonDriveTrain::AutoDriveStraightEnc(double leftCommand, double rightCommand)
{
	// if leftCommand is 0
	// not trying to move
	if(leftCommand == 0)
		// Set encoder velocity difference to 0
		encVelDiff = 0;
	// if leftCommand is not 0
	else
		// Set encoder velocity difference to right vel. - left vel.
		encVelDiff = abs(m_rightEncoderVel) - abs(m_leftEncoderVel);

	// if encoder velocity difference is greater than 100
	// right velocity faster than left
	if (encVelDiff > 100)
	{
		m_leftMasterDrive.Set(-((leftCommand * DRIVE_MAX_SPEED)));
		// Adjust right side to be slower
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * driveStraightAdjustment);
	} // END of if right is faster than left
	// if encoder velocity difference is less than 100
	// right velocity slower than left
	else if(encVelDiff < -100)
	{
		// Adjust left side to be slower
		m_leftMasterDrive.Set(-((leftCommand * DRIVE_MAX_SPEED * driveStraightAdjustment)));
		m_rightMasterDrive.Set((rightCommand * DRIVE_MAX_SPEED));
	} // END of if right is slower than left
	// if encoder velocity difference is between -100 to 100
	// more or less equal speeds
	else
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED);
	} // END of else equal speeds

	// Update encoder value variables and motor speeds and positions
	UpdateStats();
} // END of AutoDriveStraightEnc() function

void CANTalonDriveTrain::AutoDriveStraightGyro(double leftCommand, double rightCommand)
{
	// Read current angle from gyro
	currentAngle = m_pGyro->GetAngle();
	// Update variables on dashboard
	DriveTrainUpdateDashboard();

	// if moving forward and angle more than .5 degrees to right
	// or if moving backward and angle more than .5 degrees to left
	// Left side is fast.
	if(((currentAngle > .5) && leftCommand < 0) || ((currentAngle < -.5) && leftCommand > 0))
	{
		// Adjust left side to be slower
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * autoSpeedFactor) * driveStraightAdjustment);
		m_rightMasterDrive.Set((rightCommand) * DRIVE_MAX_SPEED * autoSpeedFactor);
		std::cout << "Left Fast\n";
	} // END of if left side fast
	// if moving forward and angle more than .5 degrees to left
	// or if moving backward and angle more than .5 degrees to right
	// Right side is fast.
	else if(((currentAngle < -.5) && leftCommand < 0) || ((currentAngle > .5) && leftCommand > 0))
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * autoSpeedFactor));
		// Adjust right side to be slower
		m_rightMasterDrive.Set((rightCommand * driveStraightAdjustment) * DRIVE_MAX_SPEED * autoSpeedFactor);
		std::cout << "Right Fast\n";
	} // END of if right side fast
	// if straight
	else
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * autoSpeedFactor));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * autoSpeedFactor);
	} // END of if straight

	// Update variables on dashboard
	DriveTrainUpdateDashboard();
} // END of AutoDriveStraightGyro() function


void CANTalonDriveTrain::AutoCalculateTurn(double desiredAngle, double turnSpeed)
{
	// Use desired angle to calculate positive or negative
	calculatedSpeed = turnSpeed * ((desiredAngle < 0) ? -DRIVE_MAX_SPEED * autoSpeedFactor : DRIVE_MAX_SPEED * autoSpeedFactor);
	// Reset gyro
	m_pGyro->Reset();
} // END of AutoCalculateTurn() function

bool CANTalonDriveTrain::AutoTurn(double desiredAngle)
{
	// Gyro is reset before this function is called

	// Put the desired angle on dashboard
	frc::SmartDashboard::PutNumber("Desired Angle  : ", desiredAngle);

	// Read current angle from gyro
	currentAngle = m_pGyro->GetAngle();
	// Update variables on dashboard
	DriveTrainUpdateDashboard();

	if (abs(trunc(currentAngle)) < abs(desiredAngle))
	{
		m_leftMasterDrive.Set(calculatedSpeed);
		m_rightMasterDrive.Set(calculatedSpeed);

		DriveTrainUpdateDashboard();
		return false;
	}

	Stop();
	return true;
}

bool CANTalonDriveTrain::AutoTurnCorrect(double desiredAngle)
{
	currentAngle = m_pGyro->GetAngle();
	angleLeftToTurn = desiredAngle - currentAngle;
	UpdateStats();
	DriveTrainUpdateDashboard();
	if(abs(angleLeftToTurn) > turnError)
	{
		if((abs(desiredAngle) - abs(currentAngle)) > 0)
		{
			m_leftMasterDrive.Set(calculatedSpeed);
			m_rightMasterDrive.Set(calculatedSpeed);
		}
		else
		{
			m_leftMasterDrive.Set(-calculatedSpeed);
			m_rightMasterDrive.Set(-calculatedSpeed);
		}
		return false;
	}

	Stop();
	return true;
}

bool CANTalonDriveTrain::AutoMove(double desiredRevolutions, double leftSpeed, double rightSpeed)
{
	frc::SmartDashboard::PutNumber("Desired Revolutions  : ", desiredRevolutions);
	DriveTrainUpdateDashboard();
	// +4000 attempts to make the wheel stop a little before it gets to the desired spot
	revolutionsDone = (static_cast<double>(abs(m_leftMasterDrive.GetEncPosition())) + 4000) / static_cast<double>(DRIVE_ENCDR_STEPS * 4);
	UpdateStats();
	if((abs(revolutionsDone)) < desiredRevolutions)
	{
		AutoDriveStraightGyro(-leftSpeed, -rightSpeed);

		UpdateStats();
		DriveTrainUpdateDashboard();
		return false;
	}

	Stop();
	return true;
}

void CANTalonDriveTrain::DriveTrainUpdateDashboard(void)
{
	frc::SmartDashboard::PutNumber("Revolutions Done  : ", revolutionsDone);
	frc::SmartDashboard::PutNumber("Current Angle from Drivetrain : ", currentAngle);
	frc::SmartDashboard::PutNumber("Angle Left To Turn : ", angleLeftToTurn);
}

void CANTalonDriveTrain::setBrakeMode(bool brakeOn)
{
	if(brakeOn)
	{
		m_rightMasterDrive.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		m_leftMasterDrive.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	}
	else
	{
		m_rightMasterDrive.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
		m_leftMasterDrive.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	}
}

double CANTalonDriveTrain::Deadband(double commandValue)
{
	//return commandValue;
	return (fabs(commandValue) >= DRIVE_COMMAND_DEADBAND) ? commandValue : 0.0;
}

