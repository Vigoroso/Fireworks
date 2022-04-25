#pragma once
#include "BaseSystem.h"
#include "HermiteSpline.h"
class Tank : public BaseSystem
{
public:
	Tank(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	Quaternion* getRotation();
	void setRotation(Quaternion* rot);
	void getDirec(double* p);
	void setDirec(double* p);
	void reset(double time);
	bool getHasHit();
	void setHasHit(bool h);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { m_model.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char** argv);
	HermiteSpline* getPath();

	void setVel(double vel);
	double getVel();

protected:
	GLMmodel m_model;

	float m_sx;
	float m_sy;
	float m_sz;

	double m_vel;
	Vector m_pos;
	Vector m_direc;

	HermiteSpline* m_Path;
	Quaternion m_rotation;
	bool m_hasHit;
};

