/*
 * Winch.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <Winch.h>

Winch::Winch(frc::PowerDistributionPanel* pPDP)
{
	m_pPDP = pPDP;
}

Winch::~Winch()
{
}

void Winch::Stop()
{
	m_winchMotor.Set(0.0);
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANNEL_WINCH);
}

void Winch::Update(double winchTrigger)
{
	m_winchTarget = 1 - winchTrigger;

	if (m_winchTarget > .2)
		m_winchMotor.Set(-(m_winchTarget/2));
	else
		Stop();
}

void Winch::Raise(bool bFastSpeed)
{
	if (bFastSpeed)
		m_winchMotor.Set(CLIMB_FAST_SPEED);
	else
		m_winchMotor.Set(CLIMB_SLOW_SPEED);

	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANNEL_WINCH);
}

void Winch::Lower()
{
	m_winchMotor.Set(CLIMB_SLOW_SPEED);
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANNEL_WINCH);
}

bool Winch::IsStalled()
{
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANNEL_WINCH);
	return (m_winchCurrent > STALL_CURRENT_WINCH);
}
