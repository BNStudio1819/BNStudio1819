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

int pathCount;

vec TCP_X(1, 0, 0);
vec TCP_Y(0, 1, 0);
vec TCP_Z(0, 0, -1);


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

		Matrix4 TOOL;
		TOOL.setColumn(0, TCP_X);
		TOOL.setColumn(1, TCP_Y);
		TOOL.setColumn(2, TCP_Z);
		TOOL.setColumn(3, robotpath[pathCount]);

		Nachi.inverseKinematics_analytical(TOOL, false);
		pathCount++;

		if (pathCount > MAXPOINTS) pathCount = 0;

	}
}


#endif 
