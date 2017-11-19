#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include "nachi.h"


using namespace std;
using namespace std::experimental;

#define MAXPOINTS 50


Robot_Symmetric Nachi;
vec origin;
importer path;
EndEffector EE;

vec TCP_X(1, 0, 0);
vec TCP_Y(0, 1, 0);
vec TCP_Z(0, 0, -1);

int pathCount;

void setup()
{
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	origin = Nachi.ForwardKineMatics(Nachi.rot);
	Nachi.addMeshes();

	EE = *new EndEffector("data/EE.obj");
	
	path = *new importer("data/path_tmp.txt", MAXPOINTS, 1.0);
	path.readPts_p5();

	cout << "Number of points = " << path.nCnt << endl;
	if (path.nCnt < 1) cout << "Number of points not sufficient" << endl;
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
	EE.draw();

	glPointSize(3);
	glColor3f(0, 0, 0);
	
	for (int i = 0; i < path.nCnt; i++) drawPoint(path.nodes[i].pos);

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
		TOOL.setColumn(3, path.nodes[pathCount].pos);

		Matrix4 transformMatrix = Nachi.Bars_to_world_matrices[5];
		transformMatrix.invert();

		Nachi.inverseKinematics_analytical(TOOL, false);

		for (int i = 0; i < EE.M.n_v; i++) EE.M.positions[i] = transformMatrix * EE.M.positions[i];
		pathCount++;

		if (pathCount > path.nCnt) pathCount = 0;
	}
}


#endif 
