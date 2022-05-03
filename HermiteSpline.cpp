#include "HermiteSpline.h"

HermiteSpline::HermiteSpline(const std::string& name, bool isPart2) : BaseSystem(name)
{
	m_NumControlPoints = 0;
	m_isPart2 = isPart2;

}	// SampleParticle

void HermiteSpline::getState(double* p)
{

	//VecCopy(p, m_pos);

}	// SampleParticle::getState

void HermiteSpline::setState(double* p)
{

	//VecCopy(m_pos, p);

}	// SampleParticle::setState

void HermiteSpline::reset(double time)
{
	if (!m_isPart2)
	{
		m_NumControlPoints = 0;
	}
	//setVector(m_pos, 0, 0, 0);

}	// SampleParticle::Reset


int HermiteSpline::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "set") == 0)
	{
		if (argc != 6)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			if (strcmp(argv[1], "tangent") == 0)
			{
				if (!setControlPointTangent((int)argv[2], std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5])))
				{
					animTcl::OutputMessage("system %s: bad index.", m_name.c_str());
					return TCL_ERROR;
				}
			}
			else if (strcmp(argv[1], "point") == 0)
			{
				if (!setControlPointLocation((int)argv[2], std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5])))
				{
					animTcl::OutputMessage("system %s: bad index.", m_name.c_str());
					return TCL_ERROR;
				}
			}
			else
			{
				animTcl::OutputMessage("system %s: can't set a %s.", m_name.c_str(), argv[1]);
				return TCL_ERROR;
			}
		}
	}
	else if (strcmp(argv[0], "add") == 0)
	{
		if (argc != 8)
		{
			animTcl::OutputMessage("system %s: wrong number of params for add.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			if (strcmp(argv[1], "point") == 0)
			{
				if (!addControlPoint(std::stof(argv[2]), 
					std::stof(argv[3]), 
					std::stof(argv[4]), 
					std::stof(argv[5]), 
					std::stof(argv[6]), 
					std::stof(argv[7])))
				{
					animTcl::OutputMessage("system %s: at max number of points.", m_name.c_str());
					return TCL_ERROR;
				}
			}
			else
			{
				animTcl::OutputMessage("system %s: can't add a %s.", m_name.c_str(), argv[1]);
				return TCL_ERROR;
			}
		}
	}
	else if (strcmp(argv[0], "getArcLength") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for getArcLength.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			float arcLength = getArcLength(std::stod(argv[1]));
			animTcl::OutputMessage("system %s: arc length is %f.", m_name.c_str(), arcLength);
		}
	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for load.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			readFile(argv[1]);
			animTcl::OutputMessage("loaded %s.", argv[1]);
		}
	}
	else if (strcmp(argv[0], "export") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for export.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			exportToFile(argv[1]);
			animTcl::OutputMessage("exported %s.", argv[1]);
		}
	}
	else if (strcmp(argv[0], "cr") == 0)
	{
		initCatmullRom();
	}
	glutPostRedisplay();
	return TCL_OK;

}	// SampleParticle::command

bool HermiteSpline::getPointOnCurve(Vector pos, Vector direc, double arcLength)
{
	for (int i = 1; i < (m_NumControlPoints * kNumPointsPerCurve); i++)
	{
		if (m_ArcLengths[i] >= arcLength)
		{
			int curveNumber = floor(i / kNumPointsPerCurve);
			int pointOnCurve = i % kNumPointsPerCurve;

			int prevCurveNumber = floor((i - 1) / kNumPointsPerCurve);
			int prevPointOnCurve = (i - 1) % kNumPointsPerCurve;
			VecInter(pos, 
					 m_CurvePoints[prevCurveNumber][prevPointOnCurve], 
					 m_CurvePoints[curveNumber][pointOnCurve], 
					 ((arcLength - m_ArcLengths[i - 1]) / (m_ArcLengths[i] - m_ArcLengths[i -1])));
			VecSubtract(direc, m_CurvePoints[curveNumber][pointOnCurve], m_CurvePoints[prevCurveNumber][prevPointOnCurve]);
			return true;
		}
	}
	VecCopy(pos, m_CurvePoints[m_NumControlPoints - 2][kNumPointsPerCurve - 1]);
	VecSubtract(direc, m_CurvePoints[m_NumControlPoints - 2][kNumPointsPerCurve - 1], m_CurvePoints[m_NumControlPoints - 2][kNumPointsPerCurve - 2]);
	return 0.0;
}

bool HermiteSpline::setControlPointLocation(int index, double x, double y, double z)
{
	if (index < m_NumControlPoints)
	{
		m_ControlPoints[index].SetLocation(x, y, z);
		constructCurvePointsAndArcLengths(); // ensure arc length table up to date
		return true;
	}

	return false;
}

bool HermiteSpline::setControlPointTangent(int index, double sx, double sy, double sz)
{
	if (index < m_NumControlPoints)
	{
		m_ControlPoints[index].SetTangent(sx, sy, sz);
		constructCurvePointsAndArcLengths(); // ensure arc length table up to date
		return true;
	}

	return false;
}

bool HermiteSpline::addControlPoint(double x, double y, double z, double sx, double sy, double sz)
{
	if (m_NumControlPoints < 40)
	{
		m_NumControlPoints++;
		setControlPointLocation(m_NumControlPoints - 1, x, y, z);
		setControlPointTangent(m_NumControlPoints - 1, sx, sy, sz);
		constructCurvePointsAndArcLengths(); // ensure arc length table up to date
		return true;
	}

	return false;
}

void HermiteSpline::constructCurvePointsAndArcLengths()
{
	m_ArcLengths[0] = 0.0;
	Vector* lastPoint = nullptr;
	for (int i = 0; i < m_NumControlPoints - 1; i++)
	{
		for (int j = 0; j <= kNumPointsPerCurve; j++)
		{
			setVector(m_CurvePoints[i][j], 0.0, 0.0, 0.0);
			double t = (double)j / (double)kNumPointsPerCurve;

			Vector yi;
			VecCopy(yi, *(m_ControlPoints[i].GetLocation()));
			double yiScale = (2.0 * pow(t, 3.0)) - (3.0 * pow(t, 2.0)) + 1;
			VecScale(yi, yiScale);
			VecAdd(m_CurvePoints[i][j], m_CurvePoints[i][j], yi);

			Vector yiPlus1;
			VecCopy(yiPlus1, *(m_ControlPoints[i + 1].GetLocation()));
			double yiPlus1Scale = (-2.0 * pow(t, 3.0)) + (3.0 * pow(t, 2.0));
			VecScale(yiPlus1, yiPlus1Scale);
			VecAdd(m_CurvePoints[i][j], m_CurvePoints[i][j], yiPlus1);

			Vector si;
			VecCopy (si, *(m_ControlPoints[i].GetTangent()));
			double siScale = pow(t, 3.0) + (-2.0 * pow(t, 2.0)) + t;
			VecScale(si, siScale);
			VecAdd(m_CurvePoints[i][j], m_CurvePoints[i][j], si);

			Vector siPlus1;
			VecCopy(siPlus1, *(m_ControlPoints[i + 1].GetTangent()));
			double siPlus1Scale = pow(t, 3.0) - pow(t, 2.0);
			VecScale(siPlus1, siPlus1Scale);
			VecAdd(m_CurvePoints[i][j], m_CurvePoints[i][j], siPlus1);

			if (lastPoint != nullptr)
			{
				double prevArcLength = m_ArcLengths[(i * kNumPointsPerCurve) + j - 1];
				double part1 = pow((m_CurvePoints[i][j][0] - (*lastPoint)[0]), 2);
				double part2 = pow((m_CurvePoints[i][j][1] - (*lastPoint)[1]), 2);
				double part3 = pow((m_CurvePoints[i][j][2] - (*lastPoint)[2]), 2);
				double newArcLength = sqrt(part1 + part2 + part3);
				m_ArcLengths[(i * kNumPointsPerCurve) + j] = prevArcLength + newArcLength;
			}
			lastPoint = &m_CurvePoints[i][j];
		}
	}
}

void HermiteSpline::initCatmullRom()
{
	if (m_NumControlPoints >= 3)
	{
		Vector y0Loc;
		VecCopy(y0Loc, *(m_ControlPoints[0].GetLocation()));
		Vector y1Loc;
		VecCopy(y1Loc, *(m_ControlPoints[1].GetLocation()));
		Vector y2Loc;
		VecCopy(y2Loc, *(m_ControlPoints[2].GetLocation()));
		m_ControlPoints[0].SetTangent((2.0 * (y1Loc[0] - y0Loc[0])) - ((y2Loc[0] - y0Loc[0]) / 2.0),
			(2.0 * (y1Loc[1] - y0Loc[1])) - ((y2Loc[1] - y0Loc[1]) / 2.0),
			(2.0 * (y1Loc[2] - y0Loc[2])) - ((y2Loc[2] - y0Loc[2]) / 2.0));
		//VecNormalize(*(m_ControlPoints[0].GetTangent()));
		for (int i = 1; i < m_NumControlPoints - 1; i++)
		{
			Vector yPlus1Loc;
			VecCopy(yPlus1Loc, *(m_ControlPoints[i + 1].GetLocation()));
			Vector yMinus1Loc;
			VecCopy(yMinus1Loc, *(m_ControlPoints[i - 1].GetLocation()));
			m_ControlPoints[i].SetTangent((yPlus1Loc[0] - yMinus1Loc[0]) / 2.0,
				(yPlus1Loc[1] - yMinus1Loc[1]) / 2.0,
				(yPlus1Loc[2] - yMinus1Loc[2]) / 2.0);
			//VecNormalize(*(m_ControlPoints[i].GetTangent()));
		}

		Vector ynMinus1Loc;
		VecCopy(ynMinus1Loc, *(m_ControlPoints[m_NumControlPoints - 1].GetLocation()));
		Vector ynMinus2Loc;
		VecCopy(ynMinus2Loc, *(m_ControlPoints[m_NumControlPoints - 2].GetLocation()));
		Vector ynMinus3Loc;
		VecCopy(ynMinus3Loc, *(m_ControlPoints[m_NumControlPoints - 3].GetLocation()));

		m_ControlPoints[m_NumControlPoints - 1].SetTangent((2.0 * (ynMinus2Loc[0] - ynMinus1Loc[0])) - ((ynMinus3Loc[0] - ynMinus1Loc[0]) / 2.0),
			(2.0 * (ynMinus2Loc[1] - ynMinus1Loc[1])) - ((ynMinus3Loc[1] - ynMinus1Loc[1]) / 2.0),
			(2.0 * (ynMinus2Loc[2] - ynMinus1Loc[2])) - ((ynMinus3Loc[2] - ynMinus1Loc[2]) / 2.0));
		//VecNormalize(*(m_ControlPoints[m_NumControlPoints - 1].GetTangent()));

		constructCurvePointsAndArcLengths(); // ensure arc length table up to date
	}
	else
	{
		animTcl::OutputMessage("system %s: not enough control points to perform second order accurate boundary catmull rom.", m_name.c_str());
	}
}

float HermiteSpline::getArcLength(double t)
{
	double distanceBetweenPoints = (1.0 / (((double)m_NumControlPoints - 1.0) * (double)kNumPointsPerCurve));
	int arcIndex = (int)((t / distanceBetweenPoints) + 0.5);
	return (m_ArcLengths[arcIndex]);
}

void HermiteSpline::readFile(std::string fileName)
{
	std::fstream newfile;
	newfile.open(fileName, std::fstream::in);
	std::string nextParam;
	newfile >> nextParam;
	newfile >> nextParam;

	int numPoints = std::stoi(nextParam);

	for (int i = 0; i < numPoints; i++)
	{
		newfile >> nextParam;
		double x = std::stod(nextParam);
		newfile >> nextParam;
		double y = std::stod(nextParam);
		newfile >> nextParam;
		double z = std::stod(nextParam);
		newfile >> nextParam;
		double sx = std::stod(nextParam);
		newfile >> nextParam;
		double sy = std::stod(nextParam);
		newfile >> nextParam;
		double sz = std::stod(nextParam);
		addControlPoint(x, y, z, sx, sy, sz);
	}
	constructCurvePointsAndArcLengths();
}

void HermiteSpline::exportToFile(std::string fileName)
{
	std::fstream newfile;
	newfile.open(fileName, std::fstream::out);
	newfile << m_name + " " + std::to_string(m_NumControlPoints) + "\n";
	for (int i = 0; i < m_NumControlPoints; i++)
	{
		HermiteControlPoint cp = m_ControlPoints[i];
		Vector loc;
		VecCopy(loc, *(cp.GetLocation())); 
		Vector tan;
		VecCopy(tan, *(cp.GetTangent()));
		newfile << std::to_string(loc[0]) + " " +
			std::to_string(loc[1]) + " " +
			std::to_string(loc[2]) + " " +
			std::to_string(tan[0]) + " " +
			std::to_string(tan[1]) + " " +
			std::to_string(tan[2]) + "\n";
	}
}

void HermiteSpline::display(GLenum mode)
{
	for (int i = 0; i < m_NumControlPoints - 1; i++)
	{
		glBegin(GL_LINES);
		glColor3b(1, 0, 0);
		glVertex3dv(*m_ControlPoints[i].GetLocation());
		glVertex3dv(*m_ControlPoints[i + 1].GetLocation());
		glEnd();
	}
	
	
	Vector* lastPoint = nullptr;
	for (int i = 0; i < m_NumControlPoints; i++)
	{
		for (int j = 0; j < kNumPointsPerCurve; j++)
		{
			if (lastPoint != nullptr)
			{
				glBegin(GL_LINES);
				glColor3b(1, 0, 0);
				glVertex3dv(*lastPoint);
				glVertex3dv(m_CurvePoints[i][j]);
				glEnd();
			}
			lastPoint = &m_CurvePoints[i][j];
		}
	}
	
}	// SampleParticle::display