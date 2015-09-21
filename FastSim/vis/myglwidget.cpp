// myglwidget.cpp
#include "myglwidget.h"

MyGLWidget::MyGLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    xTrans = 0.0;
    yTrans = 0.0;

    zZoom = 1.0;

    xRot = 0;
    yRot = 0;
    zRot = 0;

    texBufActive_= NULL;

    mapStaticRaw_ = NULL;
    mapStaticDist_ = NULL;
}

MyGLWidget::~MyGLWidget()
{
}

QSize MyGLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize MyGLWidget::sizeHint() const
{
    return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360)
        angle -= 360 * 16;
}

void MyGLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void MyGLWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void MyGLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

void MyGLWidget::setXTranslation(double x)
{
    if( xTrans != x ) {
        xTrans = x;
        updateGL();
    }
}

void MyGLWidget::setYTranslation(double y)
{
    if( yTrans != y ) {
        yTrans = y;
        updateGL();
    }
}

void MyGLWidget::setZZoom(double ratio)
{
    if( zZoom != ratio ) {
        zZoom = ratio;
        updateGL();
    }
}

void MyGLWidget::setMapStaticRaw(PH_MAP_UCHAR* map)
{
    if( map == NULL ) return;

    if(mapStaticRaw_ != NULL) { delete mapStaticRaw_; }
    mapStaticRaw_ = new PH_MAP_UCHAR(map->mapSize_m, map->cellSize_m, map->rowDim, map->colDim);

    *mapStaticRaw_ = *map;
    updateGL();
}

void MyGLWidget::setMapStaticDist(PH_MAP_FLOAT* map)
{
    if( map == NULL ) return;

    if(mapStaticDist_ != NULL) { delete mapStaticDist_; }
    mapStaticDist_ = new PH_MAP_FLOAT(map->mapSize_m, map->cellSize_m, map->rowDim, map->colDim);

    *mapStaticDist_ = *map;
    updateGL();
}

void MyGLWidget::initializeGL()
{
    qglClearColor(Qt::black);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    static GLfloat lightPosition[4] = { 0, 0, 10, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void MyGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(xTrans, yTrans, -10.0);

    glScalef(zZoom, zZoom, zZoom);

    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);


//    if(mapStaticRaw_ != NULL) { delete mapStaticRaw_; }
//    mapStaticRaw_ = new PH_MAP_UCHAR(100, 0.25, 400, 400);
//    mapStaticRaw_->cells[0] = 255;
//    mapStaticRaw_->cells[10] = 255;
//    mapStaticRaw_->cells[1620] = 255;
//    mapStaticRaw_->lb.x = 0;
//    mapStaticRaw_->lb.y = 0;
//    mapStaticRaw_->rt.x = mapStaticRaw_->mapSize_m;
//    mapStaticRaw_->rt.y = mapStaticRaw_->mapSize_m;

    drawOneMap(mapStaticDist_, 0, 255, 0.0);
}

void MyGLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport(0, 0, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    glOrthof(-2, +2, -2, +2, 1.0, 15.0);
#else
    glOrtho(0, +100, 0, +100, 0.0, 15.0);

    glTranslated( 0, 0, 0 );

    glRotatef(0.0, 0.0f,0.0f,1.0f);
    glScalef(1.0, 1.0, 1.0);

#endif
    glMatrixMode(GL_MODELVIEW);
}

void MyGLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void MyGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
//        setXRotation(xRot + 8 * dy);
//        setYRotation(yRot + 8 * dx);
        setXTranslation( xTrans + dx*0.1 );
        setYTranslation( yTrans - dy*0.1 );
    }
    else if (event->buttons() & Qt::RightButton) {
//        setXRotation(xRot + 8 * dy);
//        setZRotation(zRot + 8 * dx);
        if(dy > 0) { setZZoom(zZoom * 0.95); }
        else { setZZoom(zZoom * 1.05); }
    }

    lastPos = event->pos();
}

void MyGLWidget::drawOneMap( PH_MAP_UCHAR* region, unsigned char valLB, unsigned char valUB, double depthZ )
{
    if( region == NULL ) return;

    region->lb.x = 0;
    region->lb.y = 0;
    region->rt.x = region->mapSize_m;
    region->rt.y = region->mapSize_m;


    bool doAlpha = false;
    glBindTexture (GL_TEXTURE_2D, texture_);
    bool pushTexture = false;

    if (texBufActive_){
        delete [] texBufActive_;
        texBufActive_= NULL;
    }

    // if map has some dimension
    if (region->colDim > 0 && region->rowDim > 0){

        int colPadding = 0;//4 - (ppm_.vehicleOccupancyGridVector.colDim_ % 4);
        int rowPadding = 0;//4 - (ppm_.vehicleOccupancyGridVector.rowDim_ % 4);

        int colDim = region->colDim + colPadding;
        int rowDim = region->rowDim + rowPadding;

        // todo: actually use power of two or check for GL_ARB_texture_non_power_of_two
        if (!texBufActive_){
            /* allocate big enough buffer (* 4) for alpha channel
                         * whether we draw it or not.
                         */
            texBufActive_ = new uint8_t[colDim * rowDim * 4];     // for multicolors
        }

        uint8_t *bufp = texBufActive_;
        uint8_t *pend = bufp + colDim * rowDim * (doAlpha ? 4 : 3); // pointer end depends on doAlpha

        unsigned char maxDist = 0.0;
        for (int yi=0; yi<rowDim; yi++) {
            for (int xi=0; xi<colDim; xi++) {
                maxDist = fmax(maxDist, region->cells[yi*colDim + xi]);
            }
        }

        // move through each cell
        for (int yi=0; yi<rowDim; yi++) {
            for (int xi=0; xi<colDim; xi++) {
                if(bufp < pend) {

                    /**\brief optimize out unnecessary texture pushes.  is this necessary? */
#define TESTNSET(R,G,B) if(bufp[0] != (R) || bufp[1] != (G) || bufp[2] != (B) ) {bufp[0]=R; bufp[1]=G; bufp[2]=B; pushTexture = true;}
#define TESTNSETA(R,G,B,A) if(bufp[0] != (R) || bufp[1] != (G) || bufp[2] != (B) || bufp[3] != (A) ) {bufp[0]=R; bufp[1]=G; bufp[2]=B; bufp[3] = A; pushTexture = true;}

                    // if cell is empty
                    double red = valLB + (valUB - valLB) * (double)region->cells[xi*colDim + yi] / (double)maxDist;

                    unsigned char val = (unsigned char)red;
                    TESTNSET(val,0,0);
                }
                bufp+= (doAlpha ? 4 : 3); // doAlpha
            }
        } // end for loop
    }

    // set texture
    if(pushTexture) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     doAlpha ? 4 : 3,         // use 1 for white only, use 3 for colors
                     region->colDim,    // use the ScrollingByteMap's internal value of how many rows are in map
                     region->rowDim,    // viewing area is square since our scrolling map is as well.
                     0,
                     doAlpha ? GL_RGBA : GL_RGB , // doAlpha, use GL_LUMINANCE for white only, GL_RGB for colors
                     GL_UNSIGNED_BYTE,
                     texBufActive_ );
    }

    glDisable(GL_LIGHTING);

    GenericPoint3D lbPoint(region->lb.y, region->lb.x, depthZ);
    GenericPoint3D rtPoint(region->rt.y, region->rt.x, depthZ);

    GenericPoint3D ltPoint(lbPoint.x, rtPoint.y, depthZ);
    GenericPoint3D rbPoint(rtPoint.x, lbPoint.y, depthZ);

    //Draw map
    glColor3f(1.0, 1.0, 1.0);

    glEnable(GL_TEXTURE_2D);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glPushMatrix();
    // depress us to 10 cm under the world
    glTranslatef(0,0,0.1);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0); glVertexP3D(lbPoint);
    glTexCoord2f(0.0, 1.0); glVertexP3D(rbPoint);
    glTexCoord2f(1.0, 1.0); glVertexP3D(rtPoint);
    glTexCoord2f(1.0, 0.0); glVertexP3D(ltPoint);
    glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    //Draw boundary
//        glEnable(GL_LINE_SMOOTH);
    glLineWidth(1);
    glColor3ub(0xFF, 0xFF, 0xFF);
    glBegin(GL_LINE_STRIP);
    glVertexP3D(lbPoint);
    glVertexP3D(rbPoint);
    glVertexP3D(rtPoint);
    glVertexP3D(ltPoint);
    glVertexP3D(lbPoint);
    glEnd();
//        glDisable(GL_LINE_SMOOTH);

    glEnable(GL_LIGHTING);
}

void MyGLWidget::drawOneMap( PH_MAP_FLOAT* region, double valLB, double valUB, double depthZ )
{
    if( region == NULL ) return;

    region->lb.x = 0;
    region->lb.y = 0;
    region->rt.x = region->mapSize_m;
    region->rt.y = region->mapSize_m;


    bool doAlpha = false;
    glBindTexture (GL_TEXTURE_2D, texture_);
    bool pushTexture = false;

    if (texBufActive_){
        delete [] texBufActive_;
        texBufActive_= NULL;
    }

    // if map has some dimension
    if (region->colDim > 0 && region->rowDim > 0){

        int colPadding = 0;//4 - (ppm_.vehicleOccupancyGridVector.colDim_ % 4);
        int rowPadding = 0;//4 - (ppm_.vehicleOccupancyGridVector.rowDim_ % 4);

        int colDim = region->colDim + colPadding;
        int rowDim = region->rowDim + rowPadding;

        // todo: actually use power of two or check for GL_ARB_texture_non_power_of_two
        if (!texBufActive_){
            /* allocate big enough buffer (* 4) for alpha channel
                         * whether we draw it or not.
                         */
            texBufActive_ = new uint8_t[colDim * rowDim * 4];     // for multicolors
        }

        uint8_t *bufp = texBufActive_;
        uint8_t *pend = bufp + colDim * rowDim * (doAlpha ? 4 : 3); // pointer end depends on doAlpha

        double maxDist = 0.0;
        for (int yi=0; yi<rowDim; yi++) {
            for (int xi=0; xi<colDim; xi++) {
                maxDist = fmax(maxDist, region->cells[yi*colDim + xi]);
            }
        }

        // move through each cell
        for (int yi=0; yi<rowDim; yi++) {
            for (int xi=0; xi<colDim; xi++) {
                if(bufp < pend) {

                    /**\brief optimize out unnecessary texture pushes.  is this necessary? */
#define TESTNSET(R,G,B) if(bufp[0] != (R) || bufp[1] != (G) || bufp[2] != (B) ) {bufp[0]=R; bufp[1]=G; bufp[2]=B; pushTexture = true;}
#define TESTNSETA(R,G,B,A) if(bufp[0] != (R) || bufp[1] != (G) || bufp[2] != (B) || bufp[3] != (A) ) {bufp[0]=R; bufp[1]=G; bufp[2]=B; bufp[3] = A; pushTexture = true;}


                    // if cell is empty
                    double red = valLB + (valUB - valLB) * region->cells[xi*colDim + yi] / maxDist;

                    unsigned char val = (unsigned char) red;

                    TESTNSET(val,0,0);
                }
                bufp+= (doAlpha ? 4 : 3); // doAlpha
            }
        } // end for loop
    }

    // set texture
    if(pushTexture)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     doAlpha ? 4 : 3,         // use 1 for white only, use 3 for colors
                     region->colDim,    // use the ScrollingByteMap's internal value of how many rows are in map
                     region->rowDim,    // viewing area is square since our scrolling map is as well.
                     0,
                     doAlpha ? GL_RGBA : GL_RGB , // doAlpha, use GL_LUMINANCE for white only, GL_RGB for colors
                     GL_UNSIGNED_BYTE,
                     texBufActive_
                     );
    }

    glDisable(GL_LIGHTING);

    GenericPoint3D lbPoint(region->lb.y, region->lb.x, depthZ);
    GenericPoint3D rtPoint(region->rt.y, region->rt.x, depthZ);

    GenericPoint3D ltPoint(lbPoint.x, rtPoint.y, depthZ);
    GenericPoint3D rbPoint(rtPoint.x, lbPoint.y, depthZ);

    //Draw map
    glColor3f(1.0, 1.0, 1.0);

    glEnable(GL_TEXTURE_2D);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glPushMatrix();
    // depress us to 10 cm under the world
    glTranslatef(0,0,0.1);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0); glVertexP3D(lbPoint);
    glTexCoord2f(0.0, 1.0); glVertexP3D(rbPoint);
    glTexCoord2f(1.0, 1.0); glVertexP3D(rtPoint);
    glTexCoord2f(1.0, 0.0); glVertexP3D(ltPoint);
    glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    //Draw boundary
//        glEnable(GL_LINE_SMOOTH);
    glLineWidth(1);
    glColor3ub(0xFF, 0xFF, 0xFF);
    glBegin(GL_LINE_STRIP);
    glVertexP3D(lbPoint);
    glVertexP3D(rbPoint);
    glVertexP3D(rtPoint);
    glVertexP3D(ltPoint);
    glVertexP3D(lbPoint);
    glEnd();
//        glDisable(GL_LINE_SMOOTH);

    glEnable(GL_LIGHTING);
}
