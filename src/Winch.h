/*
 * Winch.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Developer
 */

#ifndef WINCH_H_
#define WINCH_H_

//#define VICTORSP
#define SPARK

#include "RobotDefs.h"

#include <VictorSP.h>
#include <Spark.h>
#include <PowerDistributionPanel.h>


#define CLIMB_SLOW_SPEED  -0.3
#define CLIMB_FAST_SPEED  -0.9



class Winch
{
private:
	frc::PowerDistributionPanel* m_pPDP;

#if defined(VICTORSP)
	frc::VictorSP m_winchMotor {0};
#elif defined(SPARK)
	frc::Spark m_winchMotor {0};
#else
#error "No Winch motor control type defined!"
#endif

	double m_winchTarget = 0;
	double m_winchCurrent = 0.0;

public:
	Winch(frc::PowerDistributionPanel* pPDP);
	virtual ~Winch();

	void Stop(void);
	void Update(double winchTriger);

	void Raise(bool bFastSpeed);
	void Lower(void);
	bool IsStalled(void);

	double GetWinchCurrent(void) {return m_winchCurrent;}
};

#endif /* WINCH_H_ */
