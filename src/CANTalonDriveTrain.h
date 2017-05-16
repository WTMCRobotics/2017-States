/*
 * CANTalonDriveTrain.h
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 asdfasdf
 */

#ifndef CAN_TALON_DRIVETRAIN_H_
#define CAN_TALON_DRIVETRAIN_H_

#include <GenericHID.h>
#include <SPI.h>

#include <CANTalon.h>
#include <XBoxController.h>
#include <ADXRS450_Gyro.h>

#include "RobotDefs.h"

// uncomment only one of these mode options
//
//#define MODE_Voltage
#define MODE_Speed
//#define MODE_Position



class CANTalonDriveTrain
{
private:
	// motor controllers
	CANTalon m_rightMasterDrive {CAN_ID_RIGHTMASTER};
	CANTalon m_rightSlaveDrive  {CAN_ID_RIGHTSLAVE};
	CANTalon m_leftSlaveDrive   {CAN_ID_LEFTSLAVE};
	CANTalon m_leftMasterDrive  {CAN_ID_LEFTMASTER};

	double m_leftTarget  = 0.0;
	double m_rightTarget = 0.0;

	double m_arcadeYAxisTarget = 0.0;

	double encVelDiff = 0.0;
	double adjustBy = driveStraightAdjustment;

	double m_leftSpeed	  	 = 0.0;
	double m_rightSpeed   	 = 0.0;
	double m_leftPosition 	 = 0.0;
	double m_rightPosition 	 = 0.0;
	double m_leftEncoderPos  = 0.0;
	double m_rightEncoderPos = 0.0;
	double m_leftEncoderVel  = 0.0;
	double m_rightEncoderVel = 0.0;

	double revolutionsDone = 0;
	double calculatedSpeed = 0;
	double currentAngle = 0;
	double angleLeftToTurn = 0;

	double m_speedFactor = .25;

	double m_startAngle		= 0.0;
	double m_endAngle		= 0.0;
	double m_deltaAngle		= 0.0;

	double m_startPosition 	= 0.0;
	double m_endPosition	= 0.0;
	double m_deltaPosition 	= 0.0;

	// pointers to global objects
	frc::XboxController* m_pController;
	frc::ADXRS450_Gyro*  m_pGyro;


public:
	CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro* pGyro);
	virtual ~CANTalonDriveTrain();

	void Stop();
	void UpdateStats(void);
	void Update(double rightCommand, double leftCommand, bool slowSpeed);
	void ArcadeDrive(double commandYAxis, double commandXAxis, bool slowSpeed);

	void AutoDriveStraightEnc(double rightCommand, double leftCommand);
	void AutoDriveStraightGyro(double rightCommand, double leftCommand);
	void AutoCalculateTurn(double desiredAngle, double turnSpeed);
	bool AutoTurn(double desiredAngle);
	bool AutoTurnCorrect(double desiredAngle);
	bool AutoMove(double desiredRevolutions, double leftSpeed, double rightSpeed);

	void SetSpeedFactor(double speedFactor) { m_speedFactor = fmax(0.0, fmin(m_speedFactor, 1.0)); }
	double GetSpeedFactor(void) { return m_speedFactor; }

	double GetEncoderVelocityDifference(void) {return encVelDiff;}

	double GetLeftSpeed(void)   	{ return m_leftSpeed;}
	double GetRightSpeed(void)  	{ return m_rightSpeed;}
	double GetLeftPosition(void) 	{ return m_leftPosition;}
	double GetRightPosition(void) 	{ return m_rightPosition;}
	double GetLeftEncoderPos(void) 	{ return m_leftEncoderPos;}
	double GetRightEncoderPos(void) { return m_rightEncoderPos;}
	double GetLeftEncoderVel(void) 	{ return m_leftEncoderVel;}
	double GetRightEncoderVel(void) { return m_rightEncoderVel;}

	void resetEncoders(void) {m_leftMasterDrive.SetPosition(0); m_rightMasterDrive.SetPosition(0);}
	void DriveTrainUpdateDashboard(void);

	void setBrakeMode(bool brakeOn);


	double GetLeftTarget(void)		{ return m_leftTarget;}
	double GetRightTarget(void) 	{ return m_rightTarget;}

	double GetStartPosition(void) 	{ return m_startPosition;}
	double GetEndPosition(void)  	{ return m_endPosition;}
	double GetDeltaPosition(void)  	{ return m_deltaPosition;}

	double GetStartAngle(void) 	{ return m_startAngle;}
	double GetEndAngle(void)  	{ return m_endAngle;}
	double GetDeltaAngle(void)  	{ return m_deltaAngle;}

private:
	double Deadband(double commandValue);

};

#endif /* CAN_TALON_DRIVETRAIN_H_ */
