/*
 * Winch.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <Winch.h>

Winch::Winch(frc::PowerDistributionPanel* pPDP)
{
	// Store passed pointer into private pointer in class
	m_pPDP = pPDP;
} // END of Winch() constructor

Winch::~Winch() {}

void Winch::Stop()
{
	// Set motor to 0
	m_winchMotor.Set(0.0);
	// Read current from the PDP
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANNEL_WINCH);
} // END of Stop() function

void Winch::Update(double winchTrigger)
{
	// Throttle control on scale from 1 to -1
	// Want on a scale from 0 to 1
	// 1 - winchTrigger = scale from 0 to 2
	// 		1 - 1 = 0; 1 - (-1) = 2
	m_winchTarget = 1 - winchTrigger;
	// Divide by 2 to get scale from 0 to 2 to be 0 to 1
	m_winchTarget = m_winchTarget/2;

	// if control is past deadband
	if (m_winchTarget > WINCH_COMMAND_DEADBAND)
		// Set motor to target
		m_winchMotor.Set(-m_winchTarget);
	// if control is not past deadband
	else
		// Stop motor
		Stop();
} // END of Update() function

// Never implemented
bool Winch::IsStalled()
{
	// Read current from PDP
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANNEL_WINCH);
	// Return true if the current is past the determined limit, else return false
	return (m_winchCurrent > STALL_CURRENT_WINCH);
} // END of IsStalled() function
