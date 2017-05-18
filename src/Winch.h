/*
 * Winch.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Developer
 */

#ifndef WINCH_H_
#define WINCH_H_

#include "RobotDefs.h"

#include <Spark.h>
#include <PowerDistributionPanel.h>

#define CLIMB_SLOW_SPEED  -0.3
#define CLIMB_FAST_SPEED  -0.9

class Winch
{
	private:
		// Initialize pointer for the power distribution module
		frc::PowerDistributionPanel* m_pPDP;

		// Initialize motor controller object to move winch
		frc::Spark m_winchMotor {0};

		// Initialize variables to store the command for the winch and the current from the PDP
		double m_winchTarget = 0;
		double m_winchCurrent = 0.0;

	public:
		// Constructor and destructor
		Winch(frc::PowerDistributionPanel* pPDP);
		virtual ~Winch();

		// Stop motor
		void Stop(void);
		// Update the winch to spin based on the Logitech controller
		void Update(double winchTriger);

		// Function to test if stalled
		// Never implemented
		bool IsStalled(void);

		// Return current
		double GetWinchCurrent(void) {return m_winchCurrent;}
};

#endif /* WINCH_H_ */
