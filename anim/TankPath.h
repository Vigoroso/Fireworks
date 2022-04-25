#ifndef _TankPath_H_
#define _TankPath_H_
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "HermiteSpline.h"
#include "Tank.h"
class TankPath : public BaseSimulator
{
public:

	TankPath(const std::string& name, Tank* target);

	int step(double time);
	int init(double time)
	{
		m_object->getState(m_pos0);
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

protected:

	Vector m_pos0; // initial position
	Vector m_pos;
	double m_vel;

	double m_currDist;

	Tank* m_object;
	HermiteSpline* m_Path;
};
#endif
