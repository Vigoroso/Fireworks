#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>

#include "shared/opengl.h"

class HermiteControlPoint
{
public:
	HermiteControlPoint(double x = 0, double y = 0, double z = 0, double sx = 0, double sy = 0, double sz = 0);
	void SetTangent(double sx, double sy, double sz);
	void SetLocation(double x, double y, double z);
	Vector* GetLocation();
	Vector* GetTangent();

protected:
	Vector m_Tangent;
	Vector m_Location;
};

