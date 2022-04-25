#include "TankPath.h"
static const double kAcceleration = 1.0;

TankPath::TankPath(const std::string& name, Tank* target) :
	BaseSimulator(name),
	m_object(target)
{
	m_currDist = 0;
}

int TankPath::step(double time)
{
	if (m_object->getHasHit())
	{
		return 0;
	}
	HermiteSpline path = *(m_object->getPath());
	double totalDist = path.getArcLength(1);
	double tenPercDist = totalDist * 0.1;

	double timeAccStops = sqrt((2 * tenPercDist) / kAcceleration);
	double constantVel = kAcceleration * timeAccStops;
	double ninetyPercDist = (totalDist - tenPercDist);

	double timeDeccStarts = (ninetyPercDist / constantVel) + (timeAccStops / 2);

	double totalTime = timeAccStops + timeDeccStarts;

	double newVelocity = 0;
	double newDist = 0;
	if (time < totalTime)
	{
		if (time < timeAccStops) // accelerating
		{
			newVelocity = constantVel * (time / timeAccStops);
			newDist = constantVel * (pow(time, 2.0) / (2.0 * timeAccStops));
		}
		else if (time >= timeDeccStarts)
		{
			newVelocity = constantVel * (1.0 - ((time - timeDeccStarts) / (timeAccStops)));
			newDist = (constantVel * (timeAccStops / 2.0)) + (constantVel * (timeDeccStarts - timeAccStops)) + (constantVel * (time - timeDeccStarts) * (1.0 - ((time - timeDeccStarts) / (2 * (timeAccStops)))));
		}
		else // constant velocity
		{
			newVelocity = constantVel;
			newDist = (constantVel * (timeAccStops / 2.0)) + (constantVel * (time - timeAccStops));
		}

		double normDist = newDist;
		Vector direc;
		path.getPointOnCurve(m_pos, direc, normDist);
		m_object->setState(m_pos);
		m_object->setVel(newVelocity);
		VecNormalize(direc);
		m_object->setDirec(direc);
		Vector axisOfRot;
		Vector origin;
		setVector(origin, 1, 0, 0);
		VecCrossProd(axisOfRot, origin, direc);

		m_object->setRotation(new Quaternion(axisOfRot[0], axisOfRot[1], axisOfRot[2], sqrt(pow(VecLength(origin), 2) * pow(VecLength(direc), 2)) + VecDotProd(origin, direc)));
	}
	else
	{
		m_object->setVel(0);
	}
	return 0;
}

int TankPath::command(int argc, myCONST_SPEC char** argv)
{
	return TCL_OK;
}
