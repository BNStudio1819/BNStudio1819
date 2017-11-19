//#define _MAIN_


#ifdef _MAIN_

#include "main.h"
#include "GeometricUtilities.h"
#include "IOUtilities.h"

using namespace Core;

Mesh mesh;
MeshFactory fac;
physics phy;
GeometricUtilities geo_util;


double planarizeForce = 0.1;
double* deviations;
bool drawMesh = true;
bool drawParticles = false;
bool run = false;

void setup()
{
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	S.addSlider(&planarizeForce, "planarizeForce");
	S.sliders[0].minVal = 0.0;
	S.sliders[0].maxVal = 1.0;

	mesh = fac.createFromOBJ("data/PlanarizationTest.obj", 1.0, true);
	
	phy = *new physics();
	phy.calcCustom = true;

	//for (int i = 0; i < mesh.n_v; i++) phy.makeParticle(mesh.positions[i], 1.0, mesh.vertices[i].onBoundary() ? 1 : 0);
	for (int i = 0; i < mesh.n_v; i++) phy.makeParticle(mesh.positions[i], false);
}


void update(int value)
{

}

void draw()
{
	vec t = geo_util.projectVector(vec(0, 0, 1), vec(1, 1, 1), vec(1, 0, 0), vec(0, 1, 0));
	drawLine(vec(1, 0, 0), t);

	backGround(0.75);
	drawGrid(50);

	mesh.draw(false);
	mesh.draw(true);
	
	phy.display(2, 5);

	setup2d();

	char info_1[20];
	glColor3f(0, 0, 0);
	sprintf(info_1, "Press E: ExportOBJ");
	drawString(info_1, 50, 300);

	char info_2[20];
	glColor3f(0, 0, 0);
	sprintf(info_2, "Press A: Run");
	drawString(info_2, 50, 325);

	restore3d();

	deviations =  new double [mesh.n_f];

	for (int i = 0; i < mesh.n_f; i++)
	{
		int *fv = mesh.faces[i].faceVertices();
		
		deviations[i] = geo_util.LineLineShortestVector(mesh.positions[fv[0]], mesh.positions[fv[2]], mesh.positions[fv[1]], mesh.positions[fv[3]]).mag();

		double deviationsMIN = MIN(pow(10,10), deviations[i]);
		double deviationsMAX = MAX(-pow(10, 10), deviations[i]);

		double rmp = ofMap(deviations[i], deviationsMIN, deviationsMAX, 0.0, 1.0);

		vec4 colorFaces = getColour(deviations[i], pow(10,10), -pow(10,10));
		glColor3f(colorFaces.r, colorFaces.g, colorFaces.b);

		//drawLine(mesh.positions[fv[0]], mesh.positions[fv[2]]);
		//drawLine(mesh.positions[fv[1]], mesh.positions[fv[3]]);
	}
}


void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'q')
	{
		for (int i = 0; i < mesh.n_v; i++)
		{
			float gaussCurv = mesh.vertices[i].gaussCurvature(mesh.positions);
			cout << gaussCurv << endl;
		}
	};

	if (k == 'a')
	{
		for (int i = 0; i < mesh.n_f; i++)
		{
			int *fv = mesh.faces[i].faceVertices();
			vec deviation = geo_util.LineLineShortestVector(mesh.positions[fv[0]], mesh.positions[fv[2]], mesh.positions[fv[1]], mesh.positions[fv[3]]);
			vec force = (deviation / deviation.mag()) * deviation.mag();

			if (deviation.mag() != 0)
			{
				phy.p[fv[0]].p += force * planarizeForce;
				phy.p[fv[2]].p += force * planarizeForce;
				phy.p[fv[1]].p -= force * planarizeForce;
				phy.p[fv[3]].p -= force * planarizeForce;
			}
		}

		for (int i = 0; i < mesh.n_v; i ++) mesh.positions[i] = phy.p[i].p;
	}

	if (k == 'e') mesh.writeOBJ("data/Planarized.obj", "Planarized", mesh.positions, false);

}


void mousePress(int b, int s, int x, int y)
{

}

void mouseMotion(int x, int y)
{

}

#endif


