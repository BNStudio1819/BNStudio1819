#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 


using namespace std;
using namespace std::experimental;

#define MAXPOINTS 50

Robot_Symmetric Nachi;
vec origin;
vec robotpath[MAXPOINTS];


void setup()
{
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	origin = Nachi.ForwardKineMatics(Nachi.rot);
	Nachi.addMeshes();

	for (int i = 0; i < MAXPOINTS; i++)
	{
		robotpath[i] = vec(5.0 * sin(2 * PI * 1 / MAXPOINTS * i), 5.0 * cos(2 * PI * 1 / MAXPOINTS * i), 0);
		robotpath[i] += vec(25, 0, 0);
	}
}

void update(int value)
{	
	Nachi.ForwardKineMatics(Nachi.rot);
}

void draw()
{
	backGround(0.75);
	drawGrid(50);

	setCamera(100, -60, -60, 10, -10);

	Nachi.draw(false);

	glPointSize(3);
	glColor3f(0, 0, 0);
	for (int i = 0; i < MAXPOINTS; i++) drawPoint(robotpath[i]);

}


void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'n')
	{
		ofstream exportfile;

		char GCode[1000];
		exportfile.open("data/MZ07-01-A.083", ios::out);
		if (exportfile.fail()) cout << "error opening the file" << "MZ07-01-A.083" << endl;

		vec TCP_X(1, 0, 0);
		vec TCP_Y(0, 1, 0);
		vec TCP_Z(0, 0, -1);

		for (int i = 0; i < MAXPOINTS; i++)
		{
			Matrix4 TOOL;
			TOOL.setColumn(0, TCP_X);
			TOOL.setColumn(1, TCP_Y);
			TOOL.setColumn(2, TCP_Z);
			TOOL.setColumn(3, robotpath[i]);

			Nachi.inverseKinematics_analytical(TOOL, false);
			Nachi.ForwardKineMatics(Nachi.rot);

			float rotationJ1, rotationJ2, rotationJ3, rotationJ4, rotationJ5, rotationJ6;

			rotationJ1 = Nachi.rot[0];
			rotationJ2 = Nachi.rot[1];
			rotationJ3 = Nachi.rot[2];
			rotationJ4 = Nachi.rot[3];
			rotationJ5 = Nachi.rot[4];
			rotationJ6 = Nachi.rot[5];

			sprintf_s(GCode, "MOVEX A=6,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),T=0.1,H=3,MS, CONF=0001", rotationJ1, rotationJ2, rotationJ3, rotationJ4, rotationJ5, rotationJ6);
			exportfile << GCode << endl;		
		}

		if (exportfile.good()) cout << "G-Code succesfully exported" << endl;

	}
}


#endif 
