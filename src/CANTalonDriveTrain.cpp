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
	m_pController = pController;
	m_pGyro		  = pGyro;

	m_leftSpeed 	= 0.0;
	m_rightSpeed 	= 0.0;
	m_speedFactor	= 1.0;

#if defined(MODE_Voltage)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPercentVbus);
	m_rightMasterDrive.Set(0);

#elif defined(MODE_Speed)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_rightMasterDrive.Set(0.0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(DRIVE_ENCDR_STEPS *  4);
	m_rightMasterDrive.ConfigNominalOutputVoltage(+0.0f, -0.0f);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMasterDrive.SetVoltageRampRate(DRIVE_RAMP_VoltsPerSec);
	m_rightMasterDrive.SetSensorDirection(true);
	m_rightMasterDrive.SetPID(DRIVE_PID_P_GAIN, DRIVE_PID_I_GAIN, DRIVE_PID_D_GAIN);

	m_leftMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_leftMasterDrive.Set(0.0);
	m_leftMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_leftMasterDrive.ConfigEncoderCodesPerRev(DRIVE_ENCDR_STEPS * 4);
	m_leftMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_leftMasterDrive.SetVoltageRampRate(DRIVE_RAMP_VoltsPerSec);
	m_leftMasterDrive.SetSensorDirection(true);
	m_leftMasterDrive.SetPID(DRIVE_PID_P_GAIN, DRIVE_PID_I_GAIN, DRIVE_PID_D_GAIN);

#elif defined(MODE_Position)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPosition);
	m_rightMasterDrive.Set(0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
#else
#error No mode selected for TallonSRX
#endif

	m_rightSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_rightSlaveDrive.Set(CAN_ID_RIGHTMASTER);

	m_leftSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_leftSlaveDrive.Set(CAN_ID_LEFTMASTER);

}

CANTalonDriveTrain::~CANTalonDriveTrain()
{

}

void CANTalonDriveTrain::UpdateStats(void)
{
	m_leftSpeed  = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	m_leftPosition  = m_leftMasterDrive.GetPosition();
	m_rightPosition = m_rightMasterDrive.GetPosition();

	m_leftEncoderPos  = m_leftMasterDrive.GetEncPosition();
	m_rightEncoderPos = m_rightMasterDrive.GetEncPosition();

	m_leftEncoderVel  = m_leftMasterDrive.GetEncVel();
	m_rightEncoderVel = m_rightMasterDrive.GetEncVel();
}

void CANTalonDriveTrain::Stop(void)
{
	m_leftSpeed = 0.0;
	m_leftMasterDrive.Set(-m_leftSpeed);
	m_rightSpeed = 0.0;
	m_rightMasterDrive.Set(m_rightSpeed);
}

void CANTalonDriveTrain::Update(double leftCommand, double rightCommand, bool slowSpeed)
{
	m_leftTarget  = Deadband(leftCommand)  * DRIVE_MAX_SPEED * m_speedFactor;
	m_rightTarget = Deadband(rightCommand) * DRIVE_MAX_SPEED * m_speedFactor;


	if (slowSpeed)
	{
		m_leftTarget  *= kSlowSpeedFactor;
		m_rightTarget *= kSlowSpeedFactor;
	}

	m_leftMasterDrive.Set(-m_leftTarget);
	m_rightMasterDrive.Set(m_rightTarget);
	UpdateStats();
}

void CANTalonDriveTrain::ArcadeDrive(double commandYAxis, double commandXAxis, bool slowSpeed)
{
	m_leftTarget  = Deadband(commandYAxis)  * DRIVE_MAX_SPEED * m_speedFactor;
	commandXAxis = Deadband(commandXAxis);

	if (slowSpeed)
	{
		m_leftTarget  *= kSlowSpeedFactor;
	}

	m_leftMasterDrive.Set(-(m_leftTarget + commandXAxis));
	m_rightMasterDrive.Set(m_leftTarget);
}

void CANTalonDriveTrain::AutoDriveStraightEnc(double leftCommand, double rightCommand)
{
	if(leftCommand == 0)
		encVelDiff = 0;
	else
		encVelDiff = abs(m_rightEncoderVel) - abs(m_leftEncoderVel);

	if (encVelDiff > 100)
	{
		m_leftMasterDrive.Set(-((leftCommand * DRIVE_MAX_SPEED * m_speedFactor)));
		m_rightMasterDrive.Set((rightCommand * DRIVE_MAX_SPEED * m_speedFactor)); // - adjustBy);
	}
	else if(encVelDiff < -100)
	{
		m_leftMasterDrive.Set(-((leftCommand * DRIVE_MAX_SPEED * m_speedFactor))); //- adjustBy));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * m_speedFactor);
	}
	// else runs if the difference is -100 to 100
	else
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * m_speedFactor));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * m_speedFactor);
	}

	UpdateStats();
}

void CANTalonDriveTrain::AutoDriveStraightGyro(double leftCommand, double rightCommand)
{
	currentAngle = m_pGyro->GetAngle();
	DriveTrainUpdateDashboard();

	if(((currentAngle > .5) && leftCommand < 0) || ((currentAngle < -.5) && leftCommand > 0))
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * autoSpeedFactor) * adjustBy);
		m_rightMasterDrive.Set((rightCommand) * DRIVE_MAX_SPEED * autoSpeedFactor);
		std::cout << "Left Fast\n";
	}
	else if(((currentAngle < -.5) && leftCommand < 0) || ((currentAngle > .5) && leftCommand > 0))
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * autoSpeedFactor));
		m_rightMasterDrive.Set((rightCommand * adjustBy) * DRIVE_MAX_SPEED * autoSpeedFactor);
		std::cout << "Right Fast\n";
	}
	else
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * autoSpeedFactor));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * autoSpeedFactor);
	}

	DriveTrainUpdateDashboard();
}


void CANTalonDriveTrain::AutoCalculateTurn(double desiredAngle, double turnSpeed)
{
	calculatedSpeed = turnSpeed * ((desiredAngle < 0) ? -DRIVE_MAX_SPEED * autoSpeedFactor : DRIVE_MAX_SPEED * autoSpeedFactor);
	m_pGyro->Reset();
}

bool CANTalonDriveTrain::AutoTurn(double desiredAngle)
{
	// Gyro is reset before this function is called
	frc::SmartDashboard::PutNumber("Desired Angle  : ", desiredAngle);
	currentAngle = m_pGyro->GetAngle();
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

