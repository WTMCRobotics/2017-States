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
	frc::VictorSP 		m_liftMotor {1};

	frc::DoubleSolenoid	m_clampSolinoid {PCM_ID, PCM_CHANNEL_GEAR_CLAMP, PCM_CHANNEL_GEAR_RELEASE};

	DigitalInput m_diGearLiftDown  {DIO_SWITCH_GEARLIFT_DOWN};
	DigitalInput m_diGearLiftUp    {DIO_SWITCH_GEARLIFT_UP};
	bool m_bGearLiftDown;
	bool m_bGearLiftUp;

	double m_gearLiftTarget = 0;

	bool m_bGearLiftClamped;
	bool m_bGearLiftStalled;

public:
	GearLift();
	virtual ~GearLift();

	void Stop(void);
	void Update(double joystickCommand, bool bClampControl, bool bOverrideLimits);

	void Raise(void);
	void Lower(void);
	void Clamp(void);
	void Release(void);

	bool IsStalled(void)	{ return m_bGearLiftStalled; }
	bool IsUp(void) 		{ return m_bGearLiftUp; }
	bool IsDown(void) 		{ return m_bGearLiftDown; }
	bool IsClamped(void) 	{ return m_bGearLiftClamped; }
};

#endif /* GEAR_LIFT_H_ */
