#include "GlutViewer.h"

#include "Rotation.h"
#include "Platform.h"

using namespace std;
using namespace alvar;
using namespace GlutViewer;

Drawable::Drawable(double _scale, double _r, double _g, double _b) {
	SetScale(_scale);
	SetColor(_r, _g, _b);
}

void Drawable::SetScale(double _scale) {
	scale=_scale;
}

void Drawable::SetColor(double _r, double _g, double _b) {
	color[0] = _r;
	color[1] = _g;
	color[2] = _b;
}

void Drawable::Draw() {
	//double color[3] = {1, 1, 1};
	glPushMatrix();
	glMultMatrixd(gl_mat);
	DrawAxis(scale, color);
	glPopMatrix();
}

void Drawable::DrawAxis(double scale, double color[3])
{
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);

	glColor3d(color[0], color[1], color[2]);
	glBegin(GL_QUADS);
		glVertex3f(-scale/2, -scale/2, 0.0);
		glVertex3f(-scale/2,  scale/2, 0.0);

		glVertex3f(-scale/2,  scale/2, 0.0);
		glVertex3f( scale/2,  scale/2, 0.0);

		glVertex3f( scale/2,  scale/2, 0.0);
		glVertex3f( scale/2, -scale/2, 0.0);
		
		glVertex3f( scale/2, -scale/2, 0.0);
		glVertex3f(-scale/2, -scale/2, 0.0);
	glEnd();

	// Z
	glColor3d(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, scale);
	glEnd();

	glDisable(GL_DEPTH_TEST);

	// X
	glColor3d(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
		glVertex3f(0.0,   0.0, 0.0);
		glVertex3f(scale, 0.0, 0.0);
	glEnd();

	// Y
	glColor3d(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
		glVertex3f(0.0, 0.0,   0.0);
		glVertex3f(0.0, scale, 0.0);
	glEnd();
}

void Drawable::SetGLMatTraQuat(double *tra, double *quat, bool flip)
{
	Rotation r;
	if (quat != 0)
	{
		CvMat cv_mat;
		cvInitMatHeader(&cv_mat, 4, 1, CV_64F, quat);
		r.SetQuaternion(&cv_mat);
	}

	int flp = 1;
	if (flip)
	{
		r.Transpose();
		//flp=-1;
	}

	CvMat cv_gl_mat;
	cvInitMatHeader(&cv_gl_mat, 4, 4, CV_64F, gl_mat); cvZero(&cv_gl_mat);
	r.GetMatrix(&cv_gl_mat);
	cvSet2D(&cv_gl_mat, 0, 3, cvScalar(flp*tra[0]));
	cvSet2D(&cv_gl_mat, 1, 3, cvScalar(flp*tra[1]));
	cvSet2D(&cv_gl_mat, 2, 3, cvScalar(flp*tra[2]));
	cvSet2D(&cv_gl_mat, 3, 3, cvScalar(1));

	cvTranspose(&cv_gl_mat, &cv_gl_mat);
}

void Drawable::SetGLMatTraRod(double *tra, double *rod)
{
	// This is the OpenGL augmentation matrix
	CvMat cv_gl_mat;
	cvInitMatHeader(&cv_gl_mat, 4, 4, CV_64F, gl_mat); cvSetIdentity(&cv_gl_mat);

	// Figure out the rotation part 
	double rot_mat_data[3][3];
	CvMat rot_mat = cvMat(3, 3, CV_64F, rot_mat_data);
	cvSetIdentity(&rot_mat);
	if (rod != 0)
	{
		CvMat rod_mat;
		cvInitMatHeader(&rod_mat, 3, 1, CV_64F, rod);
		cvRodrigues2(&rod_mat, &rot_mat, 0);
	}

	// Fill in the rotation part
	cvmSet(&cv_gl_mat, 0, 0, cvmGet(&rot_mat, 0, 0));
	cvmSet(&cv_gl_mat, 0, 1, cvmGet(&rot_mat, 0, 1));
	cvmSet(&cv_gl_mat, 0, 2, cvmGet(&rot_mat, 0, 2));
	cvmSet(&cv_gl_mat, 1, 0, cvmGet(&rot_mat, 1, 0));
	cvmSet(&cv_gl_mat, 1, 1, cvmGet(&rot_mat, 1, 1));
	cvmSet(&cv_gl_mat, 1, 2, cvmGet(&rot_mat, 1, 2));
	cvmSet(&cv_gl_mat, 2, 0, cvmGet(&rot_mat, 2, 0));
	cvmSet(&cv_gl_mat, 2, 1, cvmGet(&rot_mat, 2, 1));
	cvmSet(&cv_gl_mat, 2, 2, cvmGet(&rot_mat, 2, 2));

	// Fill in the translation part
	cvSet2D(&cv_gl_mat, 0, 3, cvScalar(tra[0]));
	cvSet2D(&cv_gl_mat, 1, 3, cvScalar(tra[1]));
	cvSet2D(&cv_gl_mat, 2, 3, cvScalar(tra[2]));

	// Transpose into OpenGL presentation order
	cvTranspose(&cv_gl_mat, &cv_gl_mat);
}

Mutex mutex_items;
vector<Drawable*> items;

int cur_button;
float elev = 0.0, azim = 0.0, rad = 0.0;
float panx = 0.0, pany = 0.0;
float jaw  = 0.0, jawx, jawy, jawz;

int ar_window;
int vr_window;

float off_x=0, off_y=0;

unsigned char* image=0;

int a_argc;
char **a_argv;
int width;
int height;

Threads threads;

double proj_mat[16];
double modelview_mat[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

static void *glut_thread(void *lpThreadParameter)
{
	//InitializeCriticalSection(&critical_section_items);

	glutInit(&a_argc, a_argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(width, height);

	ar_window = glutCreateWindow("AR");
	glutDisplayFunc(DrawAr);
	glutSpecialFunc(KeyCallback);
	glutPositionWindow(0, 0);

	vr_window = glutCreateWindow("VR");
	glutDisplayFunc(DrawVr);
	glutPositionWindow(0, height);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);

	atexit(Exit);

	glutMainLoop();
	return 0;
}

void GlutViewer::KeyCallback(int key, int x, int y)
{
	switch(key)
	{
	case GLUT_KEY_LEFT:
		off_x-=1;
		break;
	case GLUT_KEY_RIGHT:
		off_x+=1;		
		break;
	case GLUT_KEY_UP:
		off_y-=1;
		break;
	case GLUT_KEY_DOWN:
		off_y+=1;	
		break;
		
	}
}

double GlutViewer::GetXOffset()
{
	return off_x;
}

double GlutViewer::GetYOffset()
{
	return off_y;
}

void GlutViewer::Start(int argc, char** argv, int w, int h, float r)
{
	a_argc = argc;
	a_argv = argv;
	width = w;
	height = h;
    rad = r;

    threads.create(glut_thread, 0);
}

void GlutViewer::DrawFloor()
{
	glColor3f(0.5, 0.5, 1.0);

	glBegin(GL_LINES);
	for(int i = -20; i <= 20; i+=1)
	{
		glVertex3f((float)i, 0.0f,  -20);
		glVertex3f((float)i, 0.0f,   20);
		glVertex3f(-20, 0.0f, (float)i);
		glVertex3f( 20, 0.0f, (float)i);
	}
	glEnd();
}

void GlutViewer::Mouse(int button, int state, int x, int y)
{
	cur_button = button;
}

void GlutViewer::DrawAxis(float scale)
{
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(scale, 0.0, 0.0);
	glEnd();
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(0.0, scale, 0.0);
	glEnd();
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, scale);
	glEnd();
}

void GlutViewer::Motion(int x, int y)
{
	static int oldx, oldy;

	int dx = oldx-x;
	int dy = oldy-y;

	switch(cur_button)
	{
	case GLUT_LEFT_BUTTON :
		if (abs(dx)>abs(dy))
		{
			if (dx>0)
				azim += 3.0;
			else if (dx<0)
				azim -= 3.0;
		}
		else if (dy>0)
			elev -= 3.0;
		else if (dy<0)
			elev += 3.0;
		break;

	case GLUT_MIDDLE_BUTTON :
		if (abs(dx)>abs(dy))
		{
			if (dx>0)
				panx += 10.5;
			else if (dx<0)
				panx -= 10.5;
			}
			else if (dy>0)
				pany -= 10.5;
			else if (dy<0)
				pany += 10.5;
			break;//??

	case GLUT_RIGHT_BUTTON :
		if (dy > 0)
			rad += (10.2);
		else if (dy < 0)
			rad -= (10.2);
		break;

	default:
		break;
	}

	oldx = x;
	oldy = y;
}


void GlutViewer::DrawContent()
{
	DrawAxis(100.f);
	Lock lock(&mutex_items);
	for (unsigned i = 0; i < items.size(); ++i) {
		items[i]->Draw();
	}
}


void GlutViewer::DrawVr()
{
	glutSetWindow(vr_window);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glClearColor(0.5, 0.2, 0.2, 1.0);

	//glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(70, 1, 0.001, 5000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(panx, pany, -rad);
	glRotatef(-elev, 1.0, 0.0, 0.0);
	glRotatef( azim, 0.0, 1.0, 0.0);
	
	float pos[4] = {50, 0, 50};
	glLightfv(GL_LIGHT0, GL_POSITION, pos);

	DrawContent();

	//glFlush();
	glutSwapBuffers();
	glutPostRedisplay();
}

void GlutViewer::DrawAr()
{
	glutSetWindow(ar_window);
	glClearColor(0.2, 0.5, 0.2, 1.0);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	DrawVideo();

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(proj_mat);

	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(modelview_mat);

	DrawContent();

	glutSwapBuffers();
	glutPostRedisplay();
}

void GlutViewer::SetGlProjectionMatrix(double p[16]) {
	memcpy(proj_mat, p, sizeof(double)*16);
}

void GlutViewer::SetGlModelviewMatrix(double p[16]) {
	memcpy(modelview_mat, p, sizeof(double)*16);
}

void GlutViewer::Reshape(int w, int h)
{
	h = (h == 0 ?  1 : h);
	//glViewport(0, 0, w, h);

	//glMatrixMode(GL_PROJECTION);

	//glLoadIdentity();
	//glMultMatrixd(projmat);

	//glMatrixMode(GL_MODELVIEW);

	//glutPostRedisplay();
}

void GlutViewer::Exit()
{
	//DeleteCriticalSection(&critical_section_items);
}

void GlutViewer::DrawableClear() {
	Lock lock(&mutex_items);
	items.clear();
}

void GlutViewer::DrawableAdd(Drawable* item)
{
	Lock lock(&mutex_items);
	items.push_back(item);
}

// TODO ensure that threading doesn't cause any problems...
void GlutViewer::SetVideo(const IplImage* _image)
{
	image = (unsigned char*)_image->imageData;
}

void GlutViewer::DrawVideo()
{
	if(!image) return;

	glDepthMask(GL_FALSE);
	glDisable(GL_DEPTH_TEST);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glColor3f(1, 1, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 512, 512, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, image);

	glEnable(GL_TEXTURE_2D);
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, 1, 0, 1, 0, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glBegin( GL_QUADS );
		glTexCoord2d(0.0,1.0); glVertex2d(0.0,0.0);
		glTexCoord2d(1.0,1.0); glVertex2d(1.0,0.0);
		glTexCoord2d(1.0,0.0); glVertex2d(1.0,1.0);
		glTexCoord2d(0.0,0.0); glVertex2d(0.0,1.0);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

/*
void GlutViewer::Init(int argc, char** argv, int w, int h)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(w, h);
	
	ar_window = glutCreateWindow("AR");
	glutDisplayFunc(DrawAr);
	//glutReshapeFunc(Reshape);

	vr_window = glutCreateWindow("VR");
	glutDisplayFunc(DrawVr);
	//glutReshapeFunc(Reshape);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);

	atexit(Exit);
}
*/
