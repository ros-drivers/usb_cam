#ifndef _GLUTVIEWER_H_
#define _GLUTVIEWER_H_

#include <cxcore.h>

#ifdef WIN32
    #include <windows.h>
    #include <GL/gl.h>
    #include <GL/glu.h>
    #include <glut.h>
#else
    #include <GL/gl.h>
    #include <GL/glu.h>
    #include <GL/glut.h>
#endif

class Drawable
{
public:
	Drawable(double _scale=1, double _r=1, double _g=1, double _b=1);

	void SetScale(double _scale);
	void SetColor(double _r=1, double _g=1, double _b=1);
	
	virtual void Draw();
	virtual void DrawAxis(double scale, double color[3]);
	
	void SetGLMatTraQuat(double *tra, double *quat, bool flip=false);
	void SetGLMatTraRod(double *tra, double *rod);

	double scale;
	double color[3];
	double gl_mat[16];

protected:
	double ax_len;
};

namespace GlutViewer
{
	void Draw();
	void Exit();
	void Start(int argc, char** argv, int w, int h, float r=300.0);
	void Reshape(int w, int h);

	void DrawableClear();
	void DrawableAdd(Drawable* item);

	void DrawVr();
	void DrawAr();
	void DrawFloor();
	void DrawContent();
	void DrawAxis(float scale);
	void Mouse(int button, int state, int x, int y);
	void Motion(int x, int y);
	void SetGlProjectionMatrix(double p[16]);
	void SetGlModelviewMatrix(double p[16]);
	void KeyCallback(int key, int x, int y);

	void SetVideo(const IplImage* _image);
	void DrawVideo();

	double GetXOffset();
	double GetYOffset();
}

#endif
