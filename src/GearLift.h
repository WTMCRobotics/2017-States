/*
 * GearLift.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Developer
 */

#ifndef GEAR_LIFT_H_
#define GEAR_LIFT_H_

#include <VictorSP.h>
#include <DoubleSolenoid.h>
#include <DigitalInput.h>

#include "RobotDefs.h"

#define RAISE_SPEED  0.95
#define LOWER_SPEED	 0.95


class GearLift
{
	private:
		// Initialize motor controller object to move gear lift
		frc::VictorSP 		m_liftMotor {1};
		// Initialize solenoid object to open and close clamp
		frc::DoubleSolenoid	m_clampSolenoid {PCM_ID, PCM_CHANNEL_GEAR_CLAMP, PCM_CHANNEL_GEAR_RELEASE};

		// Initialize limit switch objects
		DigitalInput m_diGearLiftDown  {DIO_SWITCH_GEARLIFT_DOWN};
		DigitalInput m_diGearLiftUp    {DIO_SWITCH_GEARLIFT_UP};

		// Initialize variables to tell when switches are triggered
		bool m_bGearLiftDown;
		bool m_bGearLiftUp;

		// Initialize variable to store command from Logitech controller
		double m_gearLiftTarget = 0;

		// Initialize variable to tell when the clamp is open
		bool m_bGearLiftOpen;

		// Initialize variable to tell when gear lift has stalled
		bool m_bGearLiftStalled;

	public:
		// Constructor and destructor
		GearLift();
		virtual ~GearLift();

		// Stop motor
		void Stop(void);

		// Update gear lift up/down and open/close clamp based on Logitech controller
		void Update(double joystickCommand, bool bClampControl, bool bOverrideLimits);

		// Move gear lift up/down
		void Raise(void);
		void Lower(void);
		// Open/close clamp
		void Open(void);
		void Close(void);

		// Functions to test if stalled, up, down, or open
		bool IsStalled(void)	{ return m_bGearLiftStalled; }
		bool IsUp(void) 		{ return m_bGearLiftUp; }
		bool IsDown(void) 		{ return m_bGearLiftDown; }
		bool IsOpen(void) 		{ return m_bGearLiftOpen; }
};

#endif /* GEAR_LIFT_H_ */
