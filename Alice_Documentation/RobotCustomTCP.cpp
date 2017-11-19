#define MAIN

#ifdef MAIN

#include "main.h"

//// GLOBAL OBJECTS AND VARIABLES ////

#define MAXPOINTS 50
#define Z 18

importer ptsReader;
Robot_Symmetric Nachi;
ofstream exportfile;

vec robotOrigin;
vec robotpath[MAXPOINTS];

vec TCP_x(1, 0, 0);
vec TCP_y(0, 1, 0);
vec TCP_z(0, 0, -1);

int frm;

////////////////////////////////////////////////////////////////////////////////////////////////////////

//// MODEL

void setup()
{
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//Create sliders to control motor rotation angles
	S.addSlider(&Nachi.rot[0], "J1");
	S.addSlider(&Nachi.rot[1], "J2");
	S.addSlider(&Nachi.rot[2], "J3");
	S.addSlider(&Nachi.rot[3], "J4");
	S.addSlider(&Nachi.rot[4], "J5");
	S.addSlider(&Nachi.rot[5], "J6");

	S.sliders[0].attachToVariable(&Nachi.rot[0], -170, 170);
	S.sliders[1].attachToVariable(&Nachi.rot[1], -45, 170);
	S.sliders[2].attachToVariable(&Nachi.rot[2], -67, 120);
	S.sliders[3].attachToVariable(&Nachi.rot[3], -190, 190);
	S.sliders[4].attachToVariable(&Nachi.rot[4], -120, 120);
	S.sliders[5].attachToVariable(&Nachi.rot[5], -370, 370);

	robotOrigin = Nachi.ForwardKineMatics(Nachi.rot);
	Nachi.addMeshes();

	//Create toolpath and translate from robot origin
	for (int i = 0; i < MAXPOINTS; i++)
	{
		robotpath[i] = vec(5.0 * sin(2 * PI * 1 / MAXPOINTS * i), 5.0 * cos(2 * PI * 1 / MAXPOINTS * i), 0);
		robotpath[i] += vec(40, 0, 0);
	}

}

void update(int value)
{
	//Update FK each frame
	robotOrigin = Nachi.ForwardKineMatics(Nachi.rot);
}

//// VIEW

void draw()
{
	backGround(0.5, 0.5, 0.5);
	drawGrid(50);

	//Draw 2d text on viewport
	setup2d();

	char s[200];

	glColor3f(1, 0, 0);
	sprintf_s(s, " TCP_X %1.4f %1.4f %1.4f", Nachi.TCP_x.x, Nachi.TCP_x.y, Nachi.TCP_x.z);
	drawString(s, winW * 0.5, 25);

	glColor3f(0, 1, 0);
	sprintf_s(s, " TCP_Y %1.4f %1.4f %1.4f", Nachi.TCP_y.x, Nachi.TCP_y.y, Nachi.TCP_y.z);
	drawString(s, winW * 0.5, 50);

	glColor3f(0, 0, 1);
	sprintf_s(s, " TCP_Z %1.4f %1.4f %1.4f", Nachi.TCP_z.x, Nachi.TCP_z.y, Nachi.TCP_z.z);
	drawString(s, winW * 0.5, 75);

	glColor3f(0, 0, 0);
	sprintf(s, "Press N: Next Frame Custom");
	drawString(s, 50, 300);

	glColor3f(0, 0, 0);
	sprintf(s, "Press M: Next Frame Default");
	drawString(s, 50, 325);

	glColor3f(0, 0, 0);
	sprintf(s, "Press E: Export G-Code");
	drawString(s, 50, 350);

	restore3d();

	//Draw toolpath
	for (int i = 0; i < MAXPOINTS; i++) drawPoint(robotpath[i]);


	//Draw robot
	Nachi.draw(false);

}

//// CONTROLLERs

void keyPress(unsigned char k, int xm, int ym)
{
	// Reset robot axis
	if (k == 'r')
	{
		for (int i = 0; i < DOF; i++) Nachi.rot[i] = 0;
		Nachi.ForwardKineMatics(Nachi.rot);
	}

	// Standard 90 Orientation
	if (k == 'm')
	{
		Matrix4 TOOL;
		
		TOOL.setColumn(0, TCP_x);
		TOOL.setColumn(1, TCP_y);
		TOOL.setColumn(2, TCP_z);
		TOOL.setColumn(3, robotpath[frm] + vec (0,0,Z));
		
		Nachi.inverseKinematics_analytical(TOOL, false);
		frm++;
		if (frm > MAXPOINTS) frm = 0;
	}

	// Custom TCP calculation check Rhino file Custom_TCP
	if (k == 'n')
	{
		//Calculate new TCP location
		vec TP = vec(-6.28032, 0.101612, 20.2798);
		vec TCP = vec(4.5642, -0.00679407, 0.968496);
		vec diff = (TP - TCP);

		vec TCPtarget = robotpath[frm] + diff;

		//Calculate new TCP orientation based on 3D model mesurements
		vec x1 = vec(4.61273, 0.0764386, 0.99528);
		vec x0 = vec(4.5642, -0.00679407, 0.968496);
		vec y1 = vec(4.63664, -0.0622204, 1.00949);
		vec z1 = vec(4.51524, -0.00630462, 1.05569);

		//Normalise TCP orientation vectors since not interested into their magnitude but only direction
		vec TCP_x = (x1 - x0).normalise();
		vec TCP_y = (y1 - x0).normalise();
		vec TCP_z = (z1 - x0).normalise()*-1;

		//Construct 4x4 Matrix frame to storing position and orientation vectors
		Matrix4 TOOL;
		TOOL.setColumn(0, TCP_x);
		TOOL.setColumn(1, TCP_y);
		TOOL.setColumn(2, TCP_z);
		TOOL.setColumn(3, TCPtarget);

		//Update IK with newly computed frame 
		Nachi.inverseKinematics_analytical(TOOL, false);
		frm++;
		if (frm > MAXPOINTS) frm = 0;		
	}

	// Export G-Code file to path
	if (k == 'e')
	{
		//Create empty text file to store G-Code
		char GCode[1000];
		exportfile.open("data/MZ07-01-A.083", ios::out);
		if (exportfile.fail())cout << " error in opening file  " << "MZ07-01-A.083" << endl;

		//Calculate new TCP location
		vec TP = vec(-6.28032, 0.101612, 20.2798);
		vec TCP = vec(4.5642, -0.00679407, 0.968496);
		vec diff = (TP - TCP);

		//Calculate new TCP orientation based on 3D model mesurements
		vec x1 = vec(4.61273, 0.0764386, 0.99528);
		vec x0 = vec(4.5642, -0.00679407, 0.968496);
		vec y1 = vec(4.63664, -0.0622204, 1.00949);
		vec z1 = vec(4.51524, -0.00630462, 1.05569);

		vec TCP_x = (x1 - x0).normalise();
		vec TCP_y = (y1 - x0).normalise();
		vec TCP_z = (z1 - x0).normalise()*-1;

		//Construct 4x4 Matrix frame to storing position and orientation vectors for each vector
		for (int i = 0; i < MAXPOINTS; i++)
		{
			Matrix4 TOOL;

			TOOL.setColumn(0, TCP_x);
			TOOL.setColumn(1, TCP_y);
			TOOL.setColumn(2, TCP_z);
			TOOL.setColumn(3, robotpath[i] + diff);
			
			//Update IK with newly computed frame 
			Nachi.inverseKinematics_analytical(TOOL, false);

			//Compute FK to extract rotation angles of each motor
			Nachi.ForwardKineMatics(Nachi.rot);

			float rotationJ1, rotationJ2, rotationJ3, rotationJ4, rotationJ5, rotationJ6;

			//Store each rotation in a variable
			rotationJ1 = Nachi.rot[0];
			rotationJ2 = Nachi.rot[1];
			rotationJ3 = Nachi.rot[2];
			rotationJ4 = Nachi.rot[3];
			rotationJ5 = Nachi.rot[4];
			rotationJ6 = Nachi.rot[5];

			//Write text file with rotation angles per each motor
			sprintf_s(GCode, "MOVEX A=6,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),T=0.1,H=3,MS, CONF=0001", rotationJ1, rotationJ2, rotationJ3, rotationJ4, rotationJ5, rotationJ6);
			exportfile << GCode << endl;
		}
		if (exportfile.good())
			cout << "G-Code correctly exported" << endl;
		else cout << "G-Code not exported" << endl;
	}
}

void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{

}

#endif