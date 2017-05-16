/*
 * GearLift.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <GearLift.h>

GearLift::GearLift()
{
	// Set all boolean variables to false
	m_bGearLiftDown		= false;
	m_bGearLiftUp		= false;
	m_bGearLiftOpen		= false;
	m_bGearLiftStalled 	= false;
} // END of GearLift() constructor

GearLift::~GearLift() {}

void GearLift::Stop()
{
	// Set motor to 0
	m_liftMotor.Set(0.0);
} // END Stop() function

void GearLift::Update(double joystickCommand, bool bClampControl, bool bOverrideLimits)
{
	// The gear lift switched are N/O switches that pull the input to ground when
	//	they are closed. The state of the actual switches are inverted, since otherwise
	//	the "pull-to-ground" wiring would result in negative logic.
	//
	//	For testing purposes without the actual gear lift mechanism, the Down switch is inverted from what is should be.

	// Switches Normal Closed, which ties input to ground which results in a "0"
	// A open switch will allow the roboRio's pullup resistor to provide the "1"

	// Read limit switches
	m_bGearLiftDown = m_diGearLiftDown.Get();
	m_bGearLiftUp   = m_diGearLiftUp.Get();

	// Read joystick command and determine if it is past deadband
	// if in deadband, target = 0, else target = joystick value
	m_gearLiftTarget = (fabs(joystickCommand) >= GEAR_LIFT_COMMAND_DEADBAND) ? joystickCommand : 0.0;

	// Gear lifting logic
	// if stalled
	if (IsStalled())
	{
		// Stop motor
		Stop();
	} // END of if stalled
	// if target is greater than 0 and the upper limit switch is not triggered or the override button is pressed
	else if ((m_gearLiftTarget > 0) && (!m_bGearLiftUp || bOverrideLimits) )
	{
		// Move gear lift up
		Raise();
	} // END of if target > 0 and upper limit switch is not triggered or override is activated
	// if target is less than 0 and the lower limit switch is not triggered or the override button is pressed
	else if ((m_gearLiftTarget < 0) && (!m_bGearLiftDown || bOverrideLimits) )
	{
		// Move gear lift down
		Lower();
	} // END of if target < 0 and lower limit is not triggered or override is activated
	else
	{
		// Stop motor
		Stop();
	} // END of else

	// Gear clamping logic
	// if the clamp trigger is pressed
	if (bClampControl)
	{
		// Open the clamp and set boolean to true
		Open();
		m_bGearLiftOpen = true;
	} // END if clamp trigger pressed
	// if clamp trigger not pressed
	else
	{
		// Close the clamp and set boolean to false
		Close();
		m_bGearLiftOpen = false;
	} // END if clamp trigger not pressed
} // END Update() function

void GearLift::Raise()
{
	// Run motor to raise gear lift
	m_liftMotor.Set(-RAISE_SPEED);
} // END Raise() function

void GearLift::Lower()
{
	// Run motor to lower gear lift
	m_liftMotor.Set(LOWER_SPEED);
} // END Lower() function

void GearLift::Open()
{
	// Set solenoid to open clamp
	m_clampSolenoid.Set(DoubleSolenoid::Value::kForward);
} // END Open() function

void GearLift::Close()
{
	// Set solenoid to close clamp
	m_clampSolenoid.Set(DoubleSolenoid::Value::kReverse);
} // END Close() function
