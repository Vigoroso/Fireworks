#include "MissilePathSimulator.h"
#include "Tank.h"
#include "GlobalResourceManager.h"

MissilePathSimulator::MissilePathSimulator(const std::string& name, Missile* missile, Tank* tank) :
	BaseSimulator(name),
	m_MissileObject(missile),
	m_TankObject(tank)
{
	m_finished = false;
	m_currDist = 0;
	m_prevTime = 0;
	m_Path = new HermiteSpline("missilepath", true);
	bool success = GlobalResourceManager::use()->addSystem(m_Path, true);
	m_Path->addControlPoint(0, 0, 0, 0, 0, 0);
	m_Path->addControlPoint(0, 0, 0, 0, 0, 0);
	// make sure it was registered successfully
	assert(success);
}

int MissilePathSimulator::step(double time)
{
	if (m_MissileObject->getHasHit())
	{
		return 0;
	}
	if (time < m_prevTime)
	{
		m_prevTime = 0.0; //reset time if sim has reset
	}

	Vector missilePos;
	m_MissileObject->getState(missilePos);
	Vector tankPos;
	m_TankObject->getState(tankPos);

	Vector toHitPointFromTank;
	setVector(toHitPointFromTank, -1.0, 0, 0.4);
	Quaternion tankRotation = m_TankObject->getRotation();
	tankRotation.rotatePoint(toHitPointFromTank);
	Vector rotatedTankHitPoint;
	VecAdd(rotatedTankHitPoint, toHitPointFromTank, tankPos);

	//Quaternion missileRot = m_MissileObject->getRotation();
	//Vector missileOrientation;
	//setVector(missileOrientation, 1, 0, 0);
	//missileRot.rotatePoint(missileOrientation);
	//VecNormalize(missileOrientation);
	m_Path->setControlPointLocation(0, missilePos[0], missilePos[1], missilePos[2]);
	//m_Path->setControlPointTangent(0, missileOrientation[0], missileOrientation[1], missileOrientation[2]);

	Vector missileTan;
	VecSubtract(missileTan, rotatedTankHitPoint, missilePos);
	VecNormalize(missileTan);
	m_Path->setControlPointTangent(0, missileTan[0], missileTan[1], missileTan[2]);

	Vector tankDirec;
	m_TankObject->getDirec(tankDirec);
	VecScale(tankDirec, 5.0); // put more weight on the tank direction for a better curve
	//Quaternion tankRot = m_TankObject->getRotation();
	m_Path->setControlPointLocation(1, rotatedTankHitPoint[0], rotatedTankHitPoint[1], rotatedTankHitPoint[2]);
	m_Path->setControlPointTangent(1, tankDirec[0], tankDirec[1], tankDirec[2]);


	double totalDist = m_Path->getArcLength(1);
	double speed = m_MissileObject->getSpeed();

	double newDist = speed * (time - m_prevTime);

	Vector newPos;
	Vector direc;
	m_Path->getPointOnCurve(newPos, direc, newDist);
	m_MissileObject->setState(newPos);

	//VecNormalize(direc);
	Vector axisOfRot;
	Vector origin;
	setVector(origin, 1, 0, 0);
	VecCrossProd(axisOfRot, origin, direc);

	m_MissileObject->setRotation(new Quaternion(axisOfRot[0], axisOfRot[1], axisOfRot[2], sqrt(pow(VecLength(origin), 2) * pow(VecLength(direc), 2)) + VecDotProd(origin, direc)));

	m_prevTime = time;

	Vector posDiff;
	VecSubtract(posDiff, newPos, rotatedTankHitPoint);
	if (VecLength(posDiff) < 0.05)
	{
		m_MissileObject->setHasHit(true);
		m_TankObject->setHasHit(true);
	}
	//delete(path);
	return 0;
}

int MissilePathSimulator::command(int argc, myCONST_SPEC char** argv)
{
	return TCL_OK;
}
