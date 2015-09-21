// myglwidget.h

#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QtOpenGL>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "res/Comm.h"
#include "res/Def4PlanPerception.h"

using namespace std;

class MyGLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit MyGLWidget(QWidget *parent = 0);
    ~MyGLWidget();

    void setMapStaticRaw(PH_MAP_UCHAR* map);
    void setMapStaticDist(PH_MAP_FLOAT* map);


protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

public slots:
    // slots for xyz-rotation slider
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

    void setXTranslation(double x);
    void setYTranslation(double y);

    void setZZoom(double ratio);

signals:
    // signaling rotation from mouse movement
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

private:
    void draw();
    void drawOneMap( PH_MAP_UCHAR* region, unsigned char valLB, unsigned char valUB, double depthZ );
    void drawOneMap( PH_MAP_FLOAT* region, double valLB, double valUB, double depthZ );
        GLuint texture_;
        uint8_t *texBufActive_;

    inline void glTranslateP2D(const GenericPoint & p)   {glTranslated(p.x, p.y,  0);}
    inline void glTranslateP3D(const GenericPoint3D & p) {glTranslated(p.x, p.y,p.z);}
//    inline void glTranslateV2D(const RecVector2D &v) {glTranslated(v.x,v.y,0);}
//    inline void glTranslateV3D(const RecVector3D &v) {glTranslated(v.x,v.y,v.z);}
    inline void glVertexP2D(const GenericPoint & p)   {glVertex2d(p.x,p.y);}
    inline void glVertexP3D(const GenericPoint3D & p) {glVertex3d(p.x,p.y,p.z);}
//    inline void glVertexV3D(const RecVector3D &v) {glVertex3d(v.x,v.y,v.z);}

    double xTrans;
    double yTrans;

    double zZoom;

    int xRot;
    int yRot;
    int zRot;
    QPoint lastPos;


    PH_MAP_UCHAR* mapStaticRaw_;
    PH_MAP_FLOAT* mapStaticDist_;
};

#endif // MYGLWIDGET_H

