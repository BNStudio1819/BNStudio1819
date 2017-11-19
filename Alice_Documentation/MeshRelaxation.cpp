#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include "graph.h"

using namespace std;
using namespace std::experimental;

MeshFactory fac;
Mesh mesh, M;
Graph graph;
physics phy;

int cnt;
double restitutionLength = 0;
double displaySize = 10.0;
bool run = true;
bool display = true; 
bool exportOBJ = true; 
bool exportGraph = true;
bool displayMesh = true;


void setup()
{
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	S.addSlider(&displaySize, "displaySize");
	S.sliders[0].minVal = 0.0;
	S.sliders[0].maxVal = 20.0;

	B.addButton(&run, "Run");
	B.addButton(&display, "Display");
	B.addButton(&exportOBJ, "ExportOBJ");
	B.addButton(&exportGraph, "ExportGraph");
	B.addButton(&displayMesh, "DisplayMesh");

	mesh = fac.createFromOBJ("data/Pod.obj", 1.0, false);

	phy = *new physics(vec(0, 0, -1));
	phy.calcCustom = true;

	for (int i = 0; i < mesh.n_v; i++) graph.createVertex(mesh.positions[i]);
	for (int i = 0; i < mesh.n_e; i++) graph.createEdge(*mesh.edges[i].vStr, *mesh.edges[i].vEnd);

	for (int i = 0; i < graph.n_v; i++) phy.makeParticle(vec(mesh.positions[i].x, mesh.positions[i].y, mesh.positions[i].z), 1.0, mesh.vertices[i].onBoundary() ? 1 : 0);
	for (int i = 0; i < graph.n_e; i++) phy.makeSpring(graph.edges[i].vStr->id, graph.edges[i].vEnd->id, restitutionLength, 0.1, 1.0);
	
}

void update(int value)
{	
	if (!run)
	{
		phy.UpdateParticles(1.0, 2);

		for (int i = 0; i < mesh.n_v; i++) mesh.positions[i] = phy.p[i].p;		
	}

	if (!exportOBJ) mesh.writeOBJ("data/Relaxed.obj", "Relaxed", mesh.positions, false);

	if (!exportGraph) graph.writeGraph(1.0, "data/GraphRelaxed");

}

void draw()
{
	backGround(0.75);
	drawGrid(50);
	
	//setCamera(100, -70, 0, 0, 0);

	if (!displayMesh) mesh.draw(true);

	if (!display) phy.display(1, displaySize, false, false);

}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 's') setup();

	if (k == 'e')
	{

	}

}


void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{

}



#endif 
