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
		// Initialize motor controller objects for moving the robot
		CANTalon m_rightMasterDrive {CAN_ID_RIGHTMASTER};
		CANTalon m_rightSlaveDrive  {CAN_ID_RIGHTSLAVE};
		CANTalon m_leftSlaveDrive   {CAN_ID_LEFTSLAVE};
		CANTalon m_leftMasterDrive  {CAN_ID_LEFTMASTER};

		// Initialize pointers for the Xbox controller and the gyro
		frc::XboxController* m_pController;
		frc::ADXRS450_Gyro*  m_pGyro;

		// Initialize variables to store the command for both the left and right side
		double m_leftTarget  = 0.0;
		double m_rightTarget = 0.0;

		// Initialize variables to store encoder velocity difference
		double encVelDiff = 0.0;

		// Initialize variables to store speed, position, encoder position, and encoder velocity
		// for both the left and right side
		double m_leftSpeed	  	 = 0.0;
		double m_leftPosition 	 = 0.0;
		double m_leftEncoderPos  = 0.0;
		double m_leftEncoderVel  = 0.0;
		double m_rightSpeed   	 = 0.0;
		double m_rightPosition 	 = 0.0;
		double m_rightEncoderPos = 0.0;
		double m_rightEncoderVel = 0.0;

		// Initialize variables to use in autonomous
		double revolutionsDone = 0;
		double calculatedSpeed = 0;
		double currentAngle = 0;
		double angleLeftToTurn = 0;

		// Initialize variable to scale speed easily
		double m_speedFactor = 1;

		// Function to test if command is past deadband
		double Deadband(double commandValue);

	public:
		// Constructor and destructor
		CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro* pGyro);
		virtual ~CANTalonDriveTrain();

		// Stop motor
		void Stop();

		// Update variables with current encoder values, position, and speed
		void UpdateStats(void);

		// Update motors based on XBox controller
		void Update(double rightCommand, double leftCommand, bool slowSpeed);

		// Functions for driving straight with encoders or with gyro
		// AutoDriveStraightEnc not implemented
		void AutoDriveStraightEnc(double rightCommand, double leftCommand);
		void AutoDriveStraightGyro(double rightCommand, double leftCommand);
		// Function that moves robot straight
		bool AutoMove(double desiredRevolutions, double leftSpeed, double rightSpeed);
		// Function for finding the value to set motors to in order to turn
		void AutoCalculateTurn(double desiredAngle, double turnSpeed);
		// Function that turns robot
		bool AutoTurn(double desiredAngle);
		// Function that corrects the robot to face the desired angle
		bool AutoTurnCorrect(double desiredAngle);

		// Returns the current encoder velocity difference
		double GetEncoderVelocityDifference(void) {return encVelDiff;}

		// Return values of the motor command, speed, position, encoder position, and encoder velocity
		// for both the left and right side
		double GetLeftTarget(void)		{ return m_leftTarget;}
		double GetLeftSpeed(void)   	{ return m_leftSpeed;}
		double GetLeftPosition(void) 	{ return m_leftPosition;}
		double GetLeftEncoderPos(void) 	{ return m_leftEncoderPos;}
		double GetLeftEncoderVel(void) 	{ return m_leftEncoderVel;}
		double GetRightTarget(void) 	{ return m_rightTarget;}
		double GetRightSpeed(void)  	{ return m_rightSpeed;}
		double GetRightPosition(void) 	{ return m_rightPosition;}
		double GetRightEncoderPos(void) { return m_rightEncoderPos;}
		double GetRightEncoderVel(void) { return m_rightEncoderVel;}

		// Function to reset encoders
		void resetEncoders(void) {m_leftMasterDrive.SetPosition(0); m_rightMasterDrive.SetPosition(0);}

		// Function to update variables on the dashboard
		void DriveTrainUpdateDashboard(void);

		// Function to turn on the brakes for the motors
		void setBrakeMode(bool brakeOn);
};

#endif /* CAN_TALON_DRIVETRAIN_H_ */
