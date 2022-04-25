#include "Tank.h"
#include <anim\GlobalResourceManager.h>

Tank::Tank(const std::string& name) : BaseSystem(name)
{
	readModel("data/tank.obj");
	m_hasHit = false;
}

void Tank::getState(double* p)
{
	VecCopy(p, m_pos);
}

void Tank::setState(double* p)
{
	VecCopy(m_pos, p);
}

Quaternion* Tank::getRotation()
{
	return &m_rotation;
}

void Tank::setRotation(Quaternion* rot)
{
	m_rotation.copy(rot);
	m_rotation.normalize();
}

void Tank::getDirec(double* p)
{
	VecCopy(p, m_direc);
}

void Tank::setDirec(double* p)
{
	VecCopy(m_direc, p);
}

void Tank::reset(double time)
{
	setVector(m_pos, 0, 0, 0);
	m_hasHit = false;
}

bool Tank::getHasHit()
{
	return m_hasHit;
}

void Tank::setHasHit(bool h)
{
	m_hasHit = h;
}

void Tank::display(GLenum mode)
{
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(m_pos[0], m_pos[1], m_pos[2]);

	double theta = (acos(m_rotation.get(3)) * 2.0) * (180.0 / PI);
	glRotated(theta, m_rotation.get(0), m_rotation.get(1), m_rotation.get(2));
	glRotated(90.0, 1, 0, 0);

	glScaled(3.0, 3.0, 3.0);

	if (m_model.numvertices > 0)
		glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	else
		glutSolidSphere(1.0, 20, 20);

	glPopMatrix();
	glPopAttrib();
}

int Tank::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for state.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			bool success;
			m_Path = new HermiteSpline("path", true);
			success = GlobalResourceManager::use()->addSystem(m_Path, true);
			m_Path->command(argc, argv);

			animTcl::OutputMessage("loaded as tankpath %s.", argv[1]);
		}
	}
	return TCL_OK;
}

HermiteSpline* Tank::getPath()
{
	return m_Path;
}

void Tank::setVel(double vel)
{
	m_vel = vel;
}

double Tank::getVel()
{
	return m_vel;
}
