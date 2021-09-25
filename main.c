/*******************************************************************
           Multi-Part Model Construction and Manipulation
********************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gl/glut.h>
#include "Vector3D.h"
#include "QuadMesh.h"


const int meshSize = 64;    // Default Mesh Size
const int vWidth = 650;     // Viewport width in pixels
const int vHeight = 500;    // Viewport height in pixels



//Grid Manimulation (Blobs)
Vector3D nPos;
GLboolean m_havePivot = GL_FALSE;
static int aButton;
static unsigned char aKey;
GLboolean bControl = GL_FALSE;
GLboolean cControl = GL_FALSE;
// Lighting/shading and material properties for drone - upcoming lecture - just copy for now

//Standard Lengths
//These Variables will be used to Scale and Translate releative to the Body(Sphere) of the Submarine
static GLfloat subLengthX = 5.0;
static GLfloat subLengthY = 1.0;
static GLfloat subLengthZ = 1.0;


//Moving Variables
//These Variables will be used to Properly move the submarine as a whole rather than piece by piece
//subX, subY and subZ are used to update the translation of the submarine
//subAng is used when updating the Y angle of the submarine
//subSpeed is used when changing the speed at which the submarine is moving
//offSet is used to control the movement of the propellars around the x axis
#define PI 3.14159265358979323846
static GLfloat subX = 0.0;
static GLfloat subY = 0.0;
static GLfloat subZ = 0.0;
static GLfloat subAng = 0.0;
float colX = 0,
colY = 0,
colZ = 0;
static GLfloat subSpeed = 0.0;
static GLfloat offSet = 0.0;
//Camera Control Variables
static GLfloat pCamX = 0,	//used for the Periscope camera
			   pCamY = 0,
			   pCamZ = 0,
			   pTcamX = 0,
			   pTcamY = 0,
			   pTcamZ = 0,
				camZ = 0;
static GLfloat perY = 0.0; //moving periscope up and down
static GLfloat perAng = 0.0;
//Rocket Variables(Torpedo)
static GLfloat rSize = 0.5;//Torpedo
static GLfloat rVelocity = 10;
static GLfloat rX = 0.0;
static GLfloat rY = 0.0;
static GLfloat rZ = 0.0;
static GLfloat rAng = 0.0;
int loopR = 0;

//Ai Variables
static GLfloat sAiX = 3.0; //Ai movement
static GLfloat sAiY = 3.0;
static GLfloat sAiZ = -5.0;
static GLfloat sAiAng = 5.0;
float mvAiX = 0,
	  mvAiY = 0,
	  mvAiZ = 0,
	  mvAiR = 0;
GLboolean aiTurn = GL_FALSE;
int loopAi = 0;
int delayAi = 2;

// Light properties
static GLfloat light_position0[] = { -6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_position1[] = { 6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

// Material properties
static GLfloat drone_mat_ambient[] = { 0.4F, 0.2F, 0.0F, 1.0F };
static GLfloat drone_mat_specular[] = { 0.1F, 0.1F, 0.0F, 1.0F };
static GLfloat drone_mat_diffuse[] = { 0.9F, 0.5F, 0.0F, 1.0F };
static GLfloat drone_mat_shininess[] = { 0.0F };

// A quad mesh representing the ground
static QuadMesh groundMesh;
//Collision
GLboolean gMeshcol = GL_FALSE;
static GLfloat gMeshb = 0;


// Structure defining a bounding box, currently unused
struct BoundingBox {
    Vector3D min;
    Vector3D max;
} BBox;
struct BoundingBox boxaiSub, 
	boxmySub,
	planeBox,
	boxObj1,
	boxObj2,
	boxObj3,
	boxObj4;

//Creating dynamic blob list
Metaballs * listB;
int blobC;

// Prototypes for functions in this module
void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void mouse(int button, int state, int x, int y);
void mouseMotionHandler(int xMouse, int yMouse);
void keyboard(unsigned char key, int x, int y);
void functionKeys(int key, int x, int y);
Vector3D ScreenToWorld(int x, int y);
void dSub();				//Draws the submarine
void subPeriscope();		//Draws the Periscope on top of the submarine
void subRocket();			//Draws the Rocket fired by the submarine
void subRshoot(int n);		//Shoots the Rocket
void dSubAI();				//Draws the ai submarine
void aiMoving(int n);		//Moves the ai submarine
void randomLocation();		//Moves the ai submarine to a random location when it dies
void terBlobs();			//Draws the blobs in the terrain
void subRandomLocation();	//Moves the submarine to a random location when it hits the terrain
void subDetection();		//Sets up the submarine boundry box
void subPlanDet1();			//subPlanDet1-4 sets up the boundry box of the blobs
void subPlanDet2();
void subPlanDet3();
void subPlanDet4();
void subPlanDet();			//Sets up the boundry box of the terrain
void drawBox();				//drawBox Creates the visible boundry box
void drawBox2();
void drawBox3();
void perAlign();			//Sets up Periscope
int main(int argc, char **argv)
{
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(vWidth, vHeight);
    glutInitWindowPosition(200, 30);
    glutCreateWindow("Assignment 2");

    // Initialize GL
    initOpenGL(vWidth, vHeight);

    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotionHandler);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(functionKeys);
	glutTimerFunc(delayAi, aiMoving, 0);
	glutTimerFunc(1, subRshoot, 0);
    // Start event loop, never returns
    glutMainLoop();

    return 0;
}


// Set up OpenGL. For viewport and projection setup see reshape(). */
void initOpenGL(int w, int h)
{
    // Set up and enable lighting
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    //glEnable(GL_LIGHT1);   // This light is currently off
	glEnable(GL_TEXTURE_2D);
	makeTextureMap();
	makeTextures();
    
	// Other OpenGL setup
    glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
    glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see 
    glClearColor(0.6F, 0.6F, 0.6F, 0.0F);  // Color and depth for glClear
    glClearDepth(1.0f);
    glEnable(GL_NORMALIZE);    // Renormalize normal vectors 
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective
	// INITIALIZE VARIABLES
	//
	
	

    // Set up ground quad mesh
    Vector3D origin = NewVector3D(-20.0f, 0.0f, 8.0f);
    Vector3D dir1v = NewVector3D(1.0f, 0.0f, 0.0f);
    Vector3D dir2v = NewVector3D(0.0f, 0.0f, -1.0f);
    groundMesh = NewQuadMesh(meshSize);
    InitMeshQM(&groundMesh, meshSize, origin, 40.0, 64.0, dir1v, dir2v);

    Vector3D ambient = NewVector3D(0.0f, 0.05f, 0.0f);
    Vector3D diffuse = NewVector3D(0.4f, 0.8f, 0.4f);
    Vector3D specular = NewVector3D(0.04f, 0.04f, 0.04f);
    SetMaterialQM(&groundMesh, ambient, diffuse, specular, 0.2);

    // Set up the bounding box of the scene
    // Currently unused. You could set up bounding boxes for your objects eventually.
   Set(&BBox.min, -8.0f, 0.0, -8.0);
   Set(&BBox.max, 8.0f, 6.0,  8.0);

	//All the blobs in the terrain
   terBlobs();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

}
//Creates the Blobs on the terrain
void terBlobs() {
	listB = NewMetaballs;
	listB[blobC].pos;
	listB[blobC].height;
	listB[blobC].width;

	listB[0].pos = NewVector3D(13, -0.2, -2.6);
	listB[0].height = 5;
	listB[0].width = 0.2;

	listB[1].pos = NewVector3D(-12, 10, -15);
	listB[1].height = 4.2;
	listB[1].width = 0.55;
	
	listB[2].pos = NewVector3D(4, -0.2, -10.0);
	listB[2].height = 4.6;
	listB[2].width = 0.3;
	
	listB[3].pos = NewVector3D(3, 0.5, -2.5);
	listB[3].height = -4;
	listB[3].width = 0.054;
	
	listB[4].pos = NewVector3D(-3, 4, 6);
	listB[4].height = 2.8;
	listB[4].width = 0.65;
}
// Callback, called whenever GLUT determines that the window should be redisplayed
// or glutPostRedisplay() has been called.
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//Puts the blobs on the mesh
	UpdateMesh(&groundMesh, listB, 4);

	
	glLoadIdentity();
	//Camera Rotation around the origin
	//Changes the location of the camera
	if (cControl) {
		gluLookAt(pCamX, pCamY, pCamZ, pTcamX+camZ, pTcamY, pTcamZ, 0.0, 1.0, 0.0);
	}
	else
	{
		gluLookAt(0.0, 18.0, 22.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	}
	//Rotate
	
	glBindTexture(GL_TEXTURE_2D, getTex(0));
	DrawMeshQM(&groundMesh, meshSize);
	glBindTexture(GL_TEXTURE_2D, getTex(1));

	// Set drone material properties
    glMaterialfv(GL_FRONT, GL_AMBIENT, drone_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, drone_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, drone_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, drone_mat_shininess);

    //Ai sub
	dSubAI();
	//Boudning box outline
	drawBox();
	drawBox2();
	drawBox3();
	//Rocket fired from the sub
	if (loopR > 0) {
		subRocket();
	}
    // ...
	//Object boundry boxes
	subPlanDet1();
	subPlanDet2();
	subPlanDet3();
	subPlanDet4();
	subPlanDet();
	//Draws the submarine
	dSub();
	
    glutSwapBuffers();   // Double buffering, swap buffers
}

//Draws the bounding box for the submarine
void drawBox() {
	glColor3f(1, 1, 1);
	glBegin(GL_LINE_LOOP);
	glVertex3f(boxaiSub.max.x, boxaiSub.max.y, boxaiSub.min.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.max.y, boxaiSub.min.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.min.y, boxaiSub.min.z);
	glVertex3f(boxaiSub.max.x, boxaiSub.min.y, boxaiSub.min.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxaiSub.max.x, boxaiSub.min.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.max.x, boxaiSub.max.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.max.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.min.y, boxaiSub.max.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxaiSub.max.x, boxaiSub.max.y, boxaiSub.min.z);
	glVertex3f(boxaiSub.max.x, boxaiSub.max.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.max.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.max.y, boxaiSub.min.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxaiSub.max.x, boxaiSub.min.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.min.y, boxaiSub.max.z);
	glVertex3f(boxaiSub.min.x, boxaiSub.min.y, boxaiSub.min.z);
	glVertex3f(boxaiSub.max.x, boxaiSub.min.y, boxaiSub.min.z);
	glEnd();
}
void drawBox2() {
	glColor3f(1, 1, 1);
	glBegin(GL_LINE_LOOP);
	glVertex3f(boxmySub.max.x, boxmySub.max.y, boxmySub.min.z);
	glVertex3f(boxmySub.min.x, boxmySub.max.y, boxmySub.min.z);
	glVertex3f(boxmySub.min.x, boxmySub.min.y, boxmySub.min.z);
	glVertex3f(boxmySub.max.x, boxmySub.min.y, boxmySub.min.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxmySub.max.x, boxmySub.min.y, boxmySub.max.z);
	glVertex3f(boxmySub.max.x, boxmySub.max.y, boxmySub.max.z);
	glVertex3f(boxmySub.min.x, boxmySub.max.y, boxmySub.max.z);
	glVertex3f(boxmySub.min.x, boxmySub.min.y, boxmySub.max.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxmySub.max.x, boxmySub.max.y, boxmySub.min.z);
	glVertex3f(boxmySub.max.x, boxmySub.max.y, boxmySub.max.z);
	glVertex3f(boxmySub.min.x, boxmySub.max.y, boxmySub.max.z);
	glVertex3f(boxmySub.min.x, boxmySub.max.y, boxmySub.min.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxmySub.max.x, boxmySub.min.y, boxmySub.max.z);
	glVertex3f(boxmySub.min.x, boxmySub.min.y, boxmySub.max.z);
	glVertex3f(boxmySub.min.x, boxmySub.min.y, boxmySub.min.z);
	glVertex3f(boxmySub.max.x, boxmySub.min.y, boxmySub.min.z);
	glEnd();
}
void drawBox3() {
	glColor3f(1, 1, 1);
	glBegin(GL_LINE_LOOP);
	glVertex3f(boxObj4.max.x, boxObj4.max.y, boxObj4.min.z);
	glVertex3f(boxObj4.min.x, boxObj4.max.y, boxObj4.min.z);
	glVertex3f(boxObj4.min.x, boxObj4.min.y, boxObj4.min.z);
	glVertex3f(boxObj4.max.x, boxObj4.min.y, boxObj4.min.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxObj4.max.x, boxObj4.min.y, boxObj4.max.z);
	glVertex3f(boxObj4.max.x, boxObj4.max.y, boxObj4.max.z);
	glVertex3f(boxObj4.min.x, boxObj4.max.y, boxObj4.max.z);
	glVertex3f(boxObj4.min.x, boxObj4.min.y, boxObj4.max.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxObj4.max.x, boxObj4.max.y, boxObj4.min.z);
	glVertex3f(boxObj4.max.x, boxObj4.max.y, boxObj4.max.z);
	glVertex3f(boxObj4.min.x, boxObj4.max.y, boxObj4.max.z);
	glVertex3f(boxObj4.min.x, boxObj4.max.y, boxObj4.min.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boxObj4.max.x, boxObj4.min.y, boxObj4.max.z);
	glVertex3f(boxObj4.min.x, boxObj4.min.y, boxObj4.max.z);
	glVertex3f(boxObj4.min.x, boxObj4.min.y, boxObj4.min.z);
	glVertex3f(boxObj4.max.x, boxObj4.min.y, boxObj4.min.z);
	glEnd();
}

//Submarine boundry box
void subDetection() {
	Set(&boxmySub.min,subX -1, subY + 3, subZ - 1);
	Set(&boxmySub.max,subX +1, subY + 5, subZ + 1);
}

//Terrain boundry boxes
void subPlanDet() {
	Set(&planeBox.min, -20, -2, -16);
	Set(&planeBox.max, 20, -1, 8);
}
void subPlanDet1() {
	Set(&boxObj1.min, 11.8, 0, -3);
	Set(&boxObj1.max, 13.8, 4.5, -1.5);
}
void subPlanDet2() {
	Set(&boxObj2.min, -13, 0, -16);
	Set(&boxObj2.max, -11, 4, -14);
}
void subPlanDet3() {
	Set(&boxObj3.min, 3, 0, -11);
	Set(&boxObj3.max, 5, 4, -9);
}
void subPlanDet4() {
	Set(&boxObj4.min, -4, 0, 5);
	Set(&boxObj4.max, -2, 2, 7);
}

void dSub()
{
	glPushMatrix();
		glTranslatef(subX, subY, subZ);
		glRotatef(subAng, 0.0, 1.0, 0.0);
	// Apply transformations to construct drone, modify this!
	//Order Top down - Body, Top, Side Panel x 2, Wings x 4, Back propeller

	//Body of the submarine
	glPushMatrix();
		glTranslatef(0.0, subLengthY*4.0, 0.0);
		glScalef(subLengthX, subLengthY, subLengthZ);
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glBindTexture(GL_TEXTURE_2D, getTex(1));

	GLUquadricObj *sphere = gluNewQuadric();

		gluQuadricDrawStyle(sphere, GLU_FILL);
		glPolygonMode(GL_FRONT, GL_FILL);
		gluQuadricNormals(sphere, GLU_SMOOTH);
		gluSphere(sphere, 1.0, 10, 10);
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
	glPopMatrix();
	//Top Design
	glPushMatrix();
		glTranslatef(0.0, subLengthY*4.8, 0.0);
		glScalef(subLengthX*0.2, subLengthY*0.3, subLengthZ*0.5);
		glutSolidSphere(1, 10, 10);
	glPopMatrix();

	glPushMatrix();
		glTranslatef(0.0, subLengthY*4.8, 0.0);
		glScalef(subLengthX*0.1, subLengthY*1.0, subLengthZ*0.1);
		glutSolidCube(1.0);
	glPopMatrix();

	//Side Right design feature
	glPushMatrix();
		glTranslatef(subLengthX*0.1, subLengthY*4.5, subLengthZ*-0.5);
		glScalef(subLengthX*0.5, subLengthY*0.25, subLengthZ*0.5);
		glutSolidSphere(1, 10, 10);
	glPopMatrix();

	//Side Left Design feature
	glPushMatrix();
		glTranslatef(subLengthX*0.1, subLengthY*4.5, subLengthZ*0.5);
		glScalef(subLengthX*0.5, subLengthY*0.25, subLengthZ*0.5);
		glutSolidSphere(1, 10, 10);
	glPopMatrix();

	//Back Cone which has the propellars mounted on its tip
	glPushMatrix();
		glTranslatef(subLengthX*-0.82, subLengthY*4.0, 0.0);
		glRotatef(90, 0.0, 0.0, 1.0);
		glRotatef(90, 0.0, 1.0, 0.0);
		glRotatef(-90, 1.0, 0.0, 0.0);
		glScalef(subLengthZ*0.25, subLengthY*0.25, subLengthX*0.1);
		glutSolidCone(2, 3, 10, 10);
	glPopMatrix();

	//Back of the submarine (wings)
	glPushMatrix();
		glTranslatef(subLengthX*-0.8, subLengthY*4.0, 0.0);
		glRotatef(90, 0.0, 0.0, 1.0);
		glRotatef(90, 0.0, 1.0, 0.0);
		glScalef(subLengthX*0.3, subLengthY*1.0, subLengthZ*0.1);
		glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
		glTranslatef(subLengthX*-0.8, subLengthY*4.0, 0.0);
		glScalef(subLengthX*0.3, subLengthY*1.2, subLengthZ*0.1);
		glutSolidCube(1.0);
	glPopMatrix();

	//Propellers
	glPushMatrix();
		glTranslatef(subLengthX*-1.1, subLengthY*4.0, 0.0);
		glRotatef(offSet, 1.0, 0.0, 0.0);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glRotatef(90.0, 0.0, 0.0, 1.0);
		glScalef(subLengthX*0.8, subLengthY*0.5, subLengthZ*0.2);
		glutSolidCube(0.3);
	glPopMatrix();

	//Periscope
	subPeriscope();

	glPushMatrix();
		glTranslatef(subLengthX*-1.1, subLengthY*4.0, 0.0);
		glRotatef(offSet, 1.0, 0.0, 0.0);
		glRotatef(90.0, 0.0, 0.0, 1.0);
		glScalef(subLengthX*0.8, subLengthY*0.5, subLengthZ*0.2);
		glutSolidCube(0.3);
	glPopMatrix();

	glPopMatrix();
}
//Submarine rockets
void subRocket() {
	glPushMatrix();
	glTranslatef(rX, rY, rZ);
	glRotatef(45.0, 0.0, 0.0, 0.0);
	glScalef(1.0, 1.0, 1.0);
	glutSolidSphere(rSize, 10, 10);
	glPopMatrix();
}
//shooting the rocket
void subRshoot(int n) {

	if (loopR > 0) {
		rSize = 0.50;
		rX += (cos(subAng * (PI / 180))) / (20 / 5)*rVelocity;
		rZ -= (sin(subAng * (PI / 180))) / (20 / 5)*rVelocity;
		if (rX > boxaiSub.min.x && 
			rX < boxaiSub.max.x && 
			rY > boxaiSub.min.y && 
			rY < boxaiSub.max.y && 
			rZ > boxaiSub.min.z && 
			rZ < boxaiSub.max.z)
		{
			loopR = 0;
			randomLocation();
		}
		else {
			loopR--;
		}
	}
	else
		rSize = 0;
	glutPostRedisplay();
	glutTimerFunc(1, subRshoot, 0);
}


void subPeriscope() {
	glPushMatrix();

	//Rotates and moves the Periscope up
	glTranslatef(0.0, perY, 0.0);
	glRotatef(perAng, 0.0, 1.0, 0.0);
	//The long stick part of the Periscope
	glPushMatrix();
	glTranslatef(0.0, 5.0, 0.0);
	glScalef(0.1, 1.0, 0.1);
	glutSolidCube(1.0);
	glPopMatrix();
	//The little scope of the periscope
	glPushMatrix();
	glTranslatef(0.1, 5.5, 0.0);
	glRotatef(90.0, 0.0, 0.0, 1.0);
	glScalef(0.1, 0.2, 0.1);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();
}

//Ai Submarine construction
void dSubAI() {
glPushMatrix();
	glTranslatef(mvAiX, mvAiY, mvAiZ);
	//Ai eyes
glPushMatrix();
	glTranslatef(sAiX, sAiY + 1, sAiZ);
	glRotatef(sAiAng, 0.0, 1.0, 0.0);
	glScalef(1.0, 1.0, 0.1);
	glutSolidCube(0.5);

glPopMatrix();
	glPushMatrix();
	glTranslatef(sAiX+1, sAiY + 1, sAiZ);
	glRotatef(sAiAng, 0.0, 1.0, 0.0);
	glScalef(1.0, 1.0, 0.1);
	glutSolidCube(0.5);
glPopMatrix();

glPushMatrix();
	glTranslatef(sAiX-1, sAiY + 1, sAiZ);
	glRotatef(sAiAng, 0.0, 1.0, 0.0);
	glScalef(1.0, 1.0, 0.1);
	glutSolidCube(0.5);
	//Ai Body
glPopMatrix();
	glPushMatrix();
	glTranslatef(sAiX, sAiY, sAiZ);
	glRotatef(sAiAng, 0.0, 1.0, 0.0);
	glScalef(2.0, 0.5, 2.0);
	glutSolidSphere(1.0, 30, 30);
glPopMatrix();
glPopMatrix();
}
//Ai respawn
void randomLocation() {
	float x = rand() % 10 - 6;
	float y = rand() % 5 + 2;
	float z = rand() % 10 - 6;
	sAiX = x;
	sAiY = y;
	sAiZ = z;
}
//Submarine respawn
void subRandomLocation() {
	float x = rand() % 10 - 6;
	float y = rand() % 5 + 2;
	float z = rand() % 10 - 6;
	subX = x;
	subY = y;
	subZ = z;
	subSpeed = 0;
}

void aiMoving(int n) {

	if (aiTurn) {
		mvAiR += 10.0;
		loopAi++;
		if (loopAi >= 8) {
			aiTurn = false;
			loopAi = 0;
		}
	}
	else
	{
		mvAiX -= (cos(mvAiR * (PI / 180))) / 20;
		mvAiZ += (sin(mvAiR * (PI / 180))) / 20;

		Set(&boxaiSub.min, mvAiX + sAiX - 2, mvAiY + sAiY - 1, mvAiZ + sAiZ - 2);
		Set(&boxaiSub.max, mvAiX + sAiX + 2, mvAiY + sAiY + 1, mvAiZ + sAiZ + 2);
		loopAi++;
		if (loopAi >= 40) {
			aiTurn = true;
			loopAi = 0;
		}
	}

	glutPostRedisplay();
	glutTimerFunc(delayAi, aiMoving, 0);
}
// Callback, called at initialization and whenever user resizes the window.
void reshape(int w, int h)
{
	// Set up viewport, projection, then change to modelview matrix mode - 
	// display function will then set up camera and do modeling transforms.
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)w / h, 0.2, 40.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Set up the camera at position (0, 6, 22) looking at the origin, up along positive y axis
	gluLookAt(0.0, 6.0, 22.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}


void smoothMove()
{
	//controls the movement of the submarine in X/Z directions
	subX += subSpeed * cos(PI / 180 * -subAng);
	subZ += subSpeed * sin(PI / 180 * -subAng);
	//controls the direction the propellar is moving
	//forward
	if (subSpeed >= 0.01) {
		offSet += subSpeed * 100;
		if (offSet > 360.0)
			offSet -= 360.0;
	}
	//Backward
	else if (subSpeed <= 0.01) {
		offSet += subSpeed * 100;
		if (offSet < 0)
			offSet += 360.0;
	}
	//Detects collision with terrain and ai
	subDetection();
	
	if (boxaiSub.max.x > boxmySub.min.x &&
		boxaiSub.min.x < boxmySub.max.x &&
		boxaiSub.max.y > boxmySub.min.y &&
		boxaiSub.min.y < boxmySub.max.y &&
		boxaiSub.max.z > boxmySub.min.z &&
		boxaiSub.min.z < boxmySub.max.z) {
		subRandomLocation();
		glutPostRedisplay();
	}
	if (planeBox.max.x > boxmySub.min.x &&
		planeBox.min.x < boxmySub.max.x &&
		planeBox.max.y > boxmySub.min.y &&
		planeBox.min.y < boxmySub.max.y &&
		planeBox.max.z > boxmySub.min.z &&
		planeBox.min.z < boxmySub.max.z) {
		subRandomLocation();
		glutPostRedisplay();
	}
	if (boxObj1.max.x > boxmySub.min.x &&
		boxObj1.min.x < boxmySub.max.x &&
		boxObj1.max.y > boxmySub.min.y &&
		boxObj1.min.y < boxmySub.max.y &&
		boxObj1.max.z > boxmySub.min.z &&
		boxObj1.min.z < boxmySub.max.z) {
		subRandomLocation();
		glutPostRedisplay();
	}
	if (boxObj2.max.x > boxmySub.min.x &&
		boxObj2.min.x < boxmySub.max.x &&
		boxObj2.max.y > boxmySub.min.y &&
		boxObj2.min.y < boxmySub.max.y &&
		boxObj2.max.z > boxmySub.min.z &&
		boxObj2.min.z < boxmySub.max.z) {
		subRandomLocation();
		glutPostRedisplay();
	}
	if (boxObj3.max.x > boxmySub.min.x &&
		boxObj3.min.x < boxmySub.max.x &&
		boxObj3.max.y > boxmySub.min.y &&
		boxObj3.min.y < boxmySub.max.y &&
		boxObj3.max.z > boxmySub.min.z &&
		boxObj3.min.z < boxmySub.max.z) {
		subRandomLocation();
		glutPostRedisplay();
	}
	if (boxObj4.max.x > boxmySub.min.x &&
		boxObj4.min.x < boxmySub.max.x &&
		boxObj4.max.y > boxmySub.min.y &&
		boxObj4.min.y < boxmySub.max.y &&
		boxObj4.max.z > boxmySub.min.z &&
		boxObj4.min.z < boxmySub.max.z) {
		subRandomLocation();
		glutPostRedisplay();
	}
	//periscope
	perAlign();

	//calls the movements every 40 milleseconds 
	glutTimerFunc(40, smoothMove, 0);
	glutPostRedisplay();
}
//periscope view
void perAlign() {
	pCamX = subX + (2 * (cos((perAng+camZ) * (PI / 180))));
	pCamY = subY + perY + 5.5;
	pCamZ = subZ + (2 * (sin((perAng+camZ) * (PI / 180))));
	pTcamX = pCamX + ((cos(perAng * (PI / 180))) * 5);
	pTcamY = pCamY - 1;
	pTcamZ = pCamZ + ((sin(perAng * (PI / 180))) * 5);
}

// Callback, handles input from the keyboard, non-arrow keys
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':				//FIRE THE ROCKET
		rX = subX;
		rY = subY+4.0;
		rZ = subZ;
		rSize = subAng;
		loopR = 50;
		break;

	case 'f':				//INCREASE THE SPEED OF THE SUB
		subSpeed += 0.01;
		smoothMove();
		glutPostRedisplay();
		break;

	case 'b':				//DECREASE THE SPEED OF THE SUB
		subSpeed -= 0.01;
		smoothMove();
		glutPostRedisplay();
		break;

	case 'v':				//STOP THE SUB FROM MOVING
		subSpeed = 0.00;
		glutPostRedisplay();
		break;

	case 'w':				//MOVE THE SUB UPWARDS
		subY += 0.2;
		glutPostRedisplay();
		break;

	case 's':				//MOVE THE SUB DOWNWARDS
		subY -= 0.2;
		glutPostRedisplay();
		break;

	case 'd':				//ROTATE THE SUB TO THE RIGHT
		subAng -= 4;
		glutPostRedisplay();
		break;

	case 'a':				//ROTATE THE SUB TO THE LEFT
		subAng += 4;
		glutPostRedisplay();
		break;

	case 'z':				//MOVE THE PERISCOPE UPWARDS
		perY += 0.1;
			if (perY > 0.8)
				perY = 0.8;
			glutPostRedisplay();
			break;
	case 'x':				//MOVE THE PERISCOPE DOWNWARDS
		perY -= 0.1;
		if (perY < 0.0)
			perY = 0.0;
		glutPostRedisplay();
		break;
	case 'n':				//ROTATE THE PERISCOPE RIGHT
		perAng -= 1;
		glutPostRedisplay();
		break;
	case 'm':				//ROTATE THE PERISCOPE LEFT
		perAng += 1;
		glutPostRedisplay();
		break;
	
		glutPostRedisplay();   // Trigger a window redisplay
	}
}
	
	// Callback, handles input from the keyboard, function and arrow keys

// Callback, handles input from the keyboard, function and arrow keys
void functionKeys(int key, int x, int y)
{
	if (key == GLUT_KEY_F1)
	{
		printf("To start the game start by moving the submarine.\n");
		printf("To increase the speed forward press <f> key.\n");
		printf("To increase the speed backwards press <b> key.\n");
		printf("To stop the submarine from moving press <v> key.\n");
		printf("To raise the submarine upwards press the <w> key.\n");
		printf("To lower the submarine press the <s> key.\n");
		printf("To rotate the submarine left press the <a> key. \n");
		printf("To rotate the submarine right press the <d> key.\n");
		printf("To shoot a rocket from the submarine press the <e> key.\n");
		printf("To switch into the pariscope mode press the <f2> key.\n");
		printf("To rotate the pariscope view and model use the <n> and <m> keys.\n");
		printf("To move the pariscope up and down use the <z> and <x> keys.\n");
		printf("To zoom in and out in the periscope view use the arrow <up> and <down> keys.\n");	
	}
	if (key == GLUT_KEY_F2)
	{
		perAlign();
		cControl = !cControl;
	}
	if (cControl)
	{
		if (key == GLUT_KEY_UP) 
		{ //zooms in and out
			camZ += 0.25;
			if (camZ > 3)
				camZ = 3;
		}
		else if (key == GLUT_KEY_DOWN) {
			camZ -= 0.25;
			if (camZ < -3)
				camZ = 3;
		}
	}
    glutPostRedisplay();   // Trigger a window redisplay
}

void mouse(int button, int state, int x, int y)
{
	aButton = button;

	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			;

		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			;
		}
		break;
	default:
		break;
	}

	glutPostRedisplay();   // Trigger a window redisplay
}

// Mouse motion callback - use only if you want to 
void mouseMotionHandler(int xMouse, int yMouse)
{
	if (aButton == GLUT_LEFT_BUTTON)
	{
		;
	}

	glutPostRedisplay();   // Trigger a window redisplay
}

//Target a point on the screen
Vector3D ScreenToWorld(int x, int y)
{
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	// Read all pixels at given screen XY from the Depth Buffer     
	glReadPixels(x, (int)(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
    // you will need to finish this if you use the mouse
    return NewVector3D((float)posX, (float)posY, (float)posZ);
}


