#include "HermiteControlPoint.h"

HermiteControlPoint::HermiteControlPoint(double x, 
	double y, 
	double z, 
	double sx, 
	double sy, 
	double sz)
{
	SetTangent(sx, sy, sz);
	SetLocation(x, y, z);
}

void HermiteControlPoint::SetTangent(double sx, double sy, double sz)
{
	setVector(m_Tangent, sx, sy, sz);
}
void HermiteControlPoint::SetLocation(double x, double y, double z)
{
	setVector(m_Location, x, y, z);
}

Vector* HermiteControlPoint::GetTangent()
{
	return &m_Tangent;
}

Vector* HermiteControlPoint::GetLocation()
{
	return &m_Location;
}
