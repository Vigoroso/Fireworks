#include "Missile.h"
#include <anim\GlobalResourceManager.h>
Missile::Missile(const std::string& name) : BaseSystem(name)
{
	readModel("data/missile.obj");
	m_vel = 4.0;
	m_hasHit = false;
}

void Missile::getState(double* p)
{
	VecCopy(p, m_pos);
}

void Missile::setState(double* p)
{
	VecCopy(m_pos, p);
}

Quaternion* Missile::getRotation()
{
	return &m_rotation;
}

void Missile::setRotation(Quaternion* rot)
{
	m_rotation.copy(rot);
	m_rotation.normalize();

}

void Missile::setSpeed(double* v)
{
	m_vel = *v;
}

void Missile::setInitSpeed()
{
	m_initVel = m_vel;
}

void Missile::getInitPos(double* p)
{
	VecCopy(p, m_initPos);
}

void Missile::setInitPos(double* p)
{
}

double Missile::getSpeed()
{
	return m_vel;
}

void Missile::reset(double time)
{
	VecCopy(m_pos, m_initPos);
	m_vel = m_initVel;
	m_hasHit = false;
}

bool Missile::getHasHit()
{
	return m_hasHit;
}

void Missile::setHasHit(bool h)
{
	m_hasHit = h;
}

void Missile::display(GLenum mode)
{
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glTranslated(m_pos[0], m_pos[1], m_pos[2]);
	double theta = (acos(m_rotation.get(3)) * 2.0) * (180.0 / PI);
	glRotated(theta, m_rotation.get(0), m_rotation.get(1), m_rotation.get(2));

	if (m_model.numvertices > 0)
		glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	else
		glutSolidSphere(1.0, 20, 20);
	glPopMatrix();
	glPopAttrib();
}

int Missile::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "state") == 0)
	{
		if (argc != 8)
		{
			animTcl::OutputMessage("system %s: wrong number of params for state.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			m_Path = new HermiteSpline("path", true);
			bool success = GlobalResourceManager::use()->addSystem(m_Path, true);

			setVector(m_pos, std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
			VecCopy(m_initPos, m_pos);
			m_rotation.set(std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7]));

			animTcl::OutputMessage("loaded as tankpath %s.", argv[1]);
		}
	}
	else if (strcmp(argv[0], "speed") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for speed.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			m_vel = std::stod(argv[1]);
			m_initVel = m_vel;
		}
	}
	return TCL_OK;
}

HermiteSpline* Missile::getPath()
{
	return m_Path;
}
