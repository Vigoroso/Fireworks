#pragma once
#include "BaseSystem.h"
#include "HermiteSpline.h"
class Missile : public BaseSystem
{
public:
	Missile(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	Quaternion* getRotation();
	void setRotation(Quaternion* rot);
	void setSpeed(double* v);
	void setInitSpeed();
	void getInitPos(double* p);
	void setInitPos(double* p);
	double getSpeed();
	void reset(double time);
	bool getHasHit();
	void setHasHit(bool h);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { m_model.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char** argv);
	HermiteSpline* getPath();

protected:
	GLMmodel m_model;

	float m_sx;
	float m_sy;
	float m_sz;

	double m_initVel;
	double m_vel;

	Vector m_pos;
	Vector m_initPos;
	HermiteSpline* m_Path;
	Quaternion m_rotation;
	bool m_hasHit;
};

