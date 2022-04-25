#pragma once
#include "BaseSystem.h"
#include "Particle.h"
#include "Spring.h"

class ParticleSystem : public BaseSystem
{
public:
	const double xRange = 0.2;
	const double yRange = 0.2;
	const double zRange = 0.2;

	const double minRange = 0.1;

	Vector Origin;

	ParticleSystem(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { m_model.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char** argv);
	void initParticleSystem(int numParticles, double x, double y, double z, double vx, double vy, double vz, double colorR, double colorG, double colorB, double lifeTime);
	void setParticle(int index, double mass, double x, double y, double z, double vx, double vy, double vz);
	void setParticleColor(int index, double r, double g, double b);
	void setAllVels(double vx, double vy, double vz);
	int getNumParticles() { return m_MaxParticles; }
	Particle* getParticles() { return m_AllParticles; }
	int MaxNumSprings;
	int NumSprings;
	Spring* Springs;

	double colorR;
	double colorG;
	double colorB;

	double defaultLifeTime;
	const double lifeTimeRange = 0.5;
protected:

	int m_MaxParticles;
	Particle* m_AllParticles;

	GLMmodel m_model;

	void GenerateCubePoint(Vector point, Vector origin, double xRange, double yRange, double zRange);
	void GenerateEllipsoidPoint(Vector point, Vector origin, double xRadius, double yRadius, double zRadius);
};

