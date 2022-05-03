#include "ParticleSystem.h"
ParticleSystem::ParticleSystem(const std::string& name) : BaseSystem(name)
{
	NumSprings = 0;
}

void ParticleSystem::getState(double* p)
{
}

void ParticleSystem::setState(double* p)
{
}

void ParticleSystem::reset(double time)
{
}

void ParticleSystem::display(GLenum mode)
{
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glColor4d(0.5, 0.5, 0.0, 1.0);
	glVertex3dv(Origin);
	glEnd();

	glPointSize(1.5);
	glBegin(GL_LINES);
	for (int i = 0; i < m_MaxParticles; i++)
	{
		double scale = 1 - pow((m_AllParticles[i].timeBeenAlive / m_AllParticles[i].particleLifeTime), 3);
		// determine color
		glColor4d(scale * m_AllParticles[i].ColorR, scale * m_AllParticles[i].ColorG, scale * m_AllParticles[i].ColorB, scale);

		// draw  motion blur line
		glBegin(GL_LINES);
		Vector pos;
		m_AllParticles[i].getPos(pos);
		glVertex3dv(pos);
		Vector vel;
		m_AllParticles[i].getVel(vel);
		VecNormalize(vel);
		VecScale(vel, 0.1);
		VecSubtract(vel, pos, vel);
		glVertex3dv(vel);
		glEnd();

		// draw point
		glBegin(GL_POINTS);
		glColor4d(3 * scale * m_AllParticles[i].ColorR, 3 * scale * m_AllParticles[i].ColorG, 3 * scale * m_AllParticles[i].ColorB, scale); // tip of each particle is lifetime ratio
		glVertex3dv(pos);
		glEnd();
	}

	for (int i = 0; i < NumSprings; i++)
	{
		glBegin(GL_LINES);
		glColor4d(1.0, 0.0, 0.0, 1.0);
		Vector pos1;
		m_AllParticles[Springs[i].Particle1].getPos(pos1);
		Vector pos2;
		m_AllParticles[Springs[i].Particle2].getPos(pos2);
		glVertex3dv(pos1);
		glVertex3dv(pos2);
		glEnd();
	}
}

int ParticleSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "dim") == 0)
	{
		if (argc != 5)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			//initParticleSystem(std::stoi(argv[1]), std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
		}
	}
	else if (strcmp(argv[0], "particle") == 0)
	{
		if (argc != 9)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			setParticle(std::stoi(argv[1]),
				std::stod(argv[2]),
				std::stod(argv[3]),
				std::stod(argv[4]),
				std::stod(argv[5]),
				std::stod(argv[6]),
				std::stod(argv[7]),
				std::stod(argv[8]));
		}
	}
	else if (strcmp(argv[0], "all_velocites") == 0)
	{
		if (argc != 4)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			setAllVels(std::stod(argv[1]),
				std::stod(argv[2]),
				std::stod(argv[3]));
		}
	}
	glutPostRedisplay();
	return TCL_OK;
}

void ParticleSystem::initParticleSystem(int numParticles, double x, double y, double z, double vx, double vy, double vz, double colorR, double colorG, double colorB, double lifeTime)
{
	defaultLifeTime = lifeTime;
	m_MaxParticles = numParticles;
	m_AllParticles = new Particle[numParticles];
	setVector(Origin, x, y, z);

	double xRadius = minRange + (xRange * (double)rand() / (double)RAND_MAX);
	double yRadius = minRange + (yRange * (double)rand() / (double)RAND_MAX);
	double zRadius = minRange + (zRange * (double)rand() / (double)RAND_MAX);

	for (int i = 0; i < m_MaxParticles; i++)
	{
		Vector pos;
		GenerateEllipsoidPoint(pos, Origin, xRadius, yRadius, zRadius);

		double plifeTime = defaultLifeTime + ((2.0 * lifeTimeRange + (double)rand()) / ((double)RAND_MAX / (2.0 * lifeTimeRange)) - lifeTimeRange);

		setParticle(i, 0.01, pos[0], pos[1], pos[2], vx, vy, vz);
		Particle* particle = &m_AllParticles[i];
		particle->particleLifeTime = plifeTime;
		particle->ColorR = colorR;
		particle->ColorG = colorG;
		particle->ColorB = colorB;
	}
}

void ParticleSystem::setParticle(int index, double mass, double x, double y, double z, double vx, double vy, double vz)
{
	Particle* particle = &m_AllParticles[index];

	particle->setMass(mass);
	particle->setPos(x, y, z);
	setVector(particle->PrevPosition, x, y, z);
	particle->setVel(vx, vy, vz);
}

void ParticleSystem::setAllVels(double vx, double vy, double vz)
{
	for (int i = 0; i < m_MaxParticles; i++)
	{
		Particle* particle = &m_AllParticles[i];
		particle->setVel(vx, vy, vz);
	}
}

void ParticleSystem::GenerateCubePoint(Vector point, Vector origin, double xRange, double yRange, double zRange)
{
	double xPos = origin[0] + ((2.0 * xRange + (double)rand()) / ((double)RAND_MAX / (2.0 * xRange)) - xRange);
	double yPos = origin[1] + ((2.0 * yRange + (double)rand()) / ((double)RAND_MAX / (2.0 * yRange)) - yRange);
	double zPos = origin[2] + ((2.0 * zRange + (double)rand()) / ((double)RAND_MAX / (2.0 * zRange)) - zRange);
	setVector(point, xPos, yPos, zPos);
}

void ParticleSystem::GenerateEllipsoidPoint(Vector point, Vector origin, double xRadius, double yRadius, double zRadius)
{
	// compute random scale
	double xScale = ((double)rand()) / ((double)RAND_MAX);
	double yScale = ((double)rand()) / ((double)RAND_MAX);
	double zScale = ((double)rand()) / ((double)RAND_MAX);

	// compute random rotation
	//double xRot = PI + ((2.0 * (2.0 * PI) + (double)rand()) / ((double)RAND_MAX / (2.0 * (2.0 * PI))) - (2.0 * PI));
	//double yRot = PI + ((2.0 * (2.0 * PI) + (double)rand()) / ((double)RAND_MAX / (2.0 * (2.0 * PI))) - (2.0 * PI));
	//double zRot = PI + ((2.0 * (2.0 * PI) + (double)rand()) / ((double)RAND_MAX / (2.0 * (2.0 * PI))) - (2.0 * PI));

	double theta = (((double)rand()) / ((double)RAND_MAX)) * PI;
	double azimuth = (((double)rand()) / ((double)RAND_MAX)) * 2.0 * PI;

	double xPos = origin[0] + (xRadius * sqrt(xScale) * sin(theta) * cos(azimuth));
	double yPos = origin[1] + (yRadius * sqrt(yScale) * sin(theta) * sin(azimuth));
	double zPos = origin[2] + (zRadius * sqrt(zScale) * cos(theta));
	setVector(point, xPos, yPos, zPos);
}
