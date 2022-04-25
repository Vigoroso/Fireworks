#pragma once
#include "BaseSystem.h"
#include "HermiteControlPoint.h"
#include <util/util.h>
#include <iostream>
#include <fstream>
#include <string>

class HermiteSpline : public BaseSystem
{
public:
	HermiteSpline( const std::string& name, bool ispart2);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { m_model.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char** argv);
	bool HermiteSpline::getPointOnCurve(Vector pos, Vector direc, double arcLength);
	float getArcLength(double t);
	bool addControlPoint(double x, double y, double z, double sx, double sy, double sz);
	bool setControlPointLocation(int index, double x, double y, double z);
	bool setControlPointTangent(int index, double sx, double sy, double sz);

protected:

	static const int kNumPointsPerCurve = 100;
	static const int kMaxControlPoints = 40;

	HermiteControlPoint m_ControlPoints[kMaxControlPoints];
	Vector m_CurvePoints[kMaxControlPoints - 1][kNumPointsPerCurve];
	double m_ArcLengths[(kMaxControlPoints - 1) * kNumPointsPerCurve];
	int m_NumControlPoints;

	GLMmodel m_model;

	void constructCurvePointsAndArcLengths();
	void initCatmullRom();
	void readFile(std::string fileName);
	void exportToFile(std::string fileName);
	bool m_isPart2;
};

