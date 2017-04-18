/*
 * GearLift.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <GearLift.h>

GearLift::GearLift()
{
	m_bGearLiftDown		= false;
	m_bGearLiftUp		= false;
	m_bGearLiftClamped	= false;
	m_bGearLiftStalled 	= false;
}

GearLift::~GearLift()
{
}

void GearLift::Stop()
{
	m_liftMotor.Set(0.0);
}

void GearLift::Update(double joystickCommand, bool bClampControl, bool bOverrideLimits)
{
	// The gear lift switched are N/O switches that pull the input to ground when
	//	they are closed. The state of the actual switches are inverted, since otherwise
	//	the "pull-to-ground" wiring would result in negative logic.
	//
	//	For testing purposes without the actual gear lift mechanism, the Down switch is inverted from what is should be.

	// Switches Normal Closed, which ties input to ground which results in a "0"
	// A open switch will allow the roboRio's pullup resistor to provide the "1"
	m_bGearLiftDown = m_diGearLiftDown.Get();
	m_bGearLiftUp   = m_diGearLiftUp.Get();

	m_gearLiftTarget = (fabs(joystickCommand) >= GEAR_LIFT_COMMAND_DEADBAND) ? joystickCommand : 0.0;

	// gear lifting logic
	//
	if (IsStalled())
	{
		Stop();
	}
	else if ((m_gearLiftTarget > 0) && (!m_bGearLiftUp || bOverrideLimits) )
	{
		Raise();
	}
	else if ((m_gearLiftTarget < 0) && (!m_bGearLiftDown || bOverrideLimits) )
	{
		Lower();
	}
	else
	{
		Stop();
	}

	// gear clamping logic
	//
	if (bClampControl)
	{
		Clamp();
		m_bGearLiftClamped = true;
	}
	else
	{
		Release();
		m_bGearLiftClamped = false;
	}
}

void GearLift::Raise()
{
	m_liftMotor.Set(-RAISE_SPEED);
}

void GearLift::Lower()
{
	m_liftMotor.Set(LOWER_SPEED);
}

void GearLift::Clamp()
{
	m_clampSolinoid.Set(DoubleSolenoid::Value::kForward);
}

void GearLift::Release()
{
	m_clampSolinoid.Set(DoubleSolenoid::Value::kReverse);
}

