#include "gl_display.h"

#include <QDebug>
#include <QThread>

namespace ProcGen {

namespace GameComponent {

  
/****************************************************************************
**
** Author: Richard Baxter
**
** Default Constructor
**
****************************************************************************/
GLDisplay::GLDisplay() : glDisplayWidget(0)
{
  //connect (&glDisplayWidget, SIGNAL (sendingContext( )), this, SLOT(sendReady()));
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GLDisplay::~GLDisplay()
{

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::setGLDisplayWidget(GLDisplayWidget* glDisplayWidget)
{
  this->glDisplayWidget = glDisplayWidget;
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::resize(int width, int height)
{
  qDebug() << "GLDisplay::resizeGL - " << width << " " << height << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, width, height);
  //glOrtho(	0, width, 0, height, -100, 100);
  gluPerspective( 75/*GLdouble	fovy*/,
			            1/*GLdouble	aspect*/,
			            0.1/*GLdouble	zNear*/,
			            10000/*GLdouble	zFar*/ );

  glMatrixMode(GL_MODELVIEW);
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::initialize() {
  qDebug() << "GLDisplay::initialize" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  
  glClearColor(0.0, 0.0, 0.0, 0.0);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_TEXTURE_2D);
  
  glMatrixMode(GL_MODELVIEW);
  qDebug() << "END GLDisplay::initialize" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::initRenderStep()
{
  //qDebug() << "GLDisplay::initRenderStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::cleanupRenderStep()
{
  //qDebug() << "GLDisplay::cleanupRenderStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::setUpCamera(double x, double y, double z, double pitch, double yaw) const
{
  glRotated(pitch/M_PI*180, 1,0,0);
  glRotated(yaw/M_PI*180, 0,1,0);
  glTranslated(x,y,z);
}



/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
unsigned int GLDisplay::loadTexture(const QImage& image) const {
  qDebug() << "GLDisplay::loadTexture";
  qDebug() << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  if (image.isNull ())
    return -1;
  else
    return glDisplayWidget->bindTexture(image);
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::bindTexture(unsigned int id) const {
  
  glBindTexture(GL_TEXTURE_2D, id);
}

/****************************************************************************
**
** Author: Richard Baxter
**
** yay for http://irrlicht.sourceforge.net/phpBB2/viewtopic.php?t=17669 and http://awakenedmmo.org/docs/physx/html/index.html
**
****************************************************************************/
void GLDisplay::drawActor(NxActor* actor) const {


  NxShape*const * shapes = actor->getShapes();
  NxU32 nShapes = actor->getNbShapes();
  nShapes = actor->getNbShapes();
  while (nShapes--)
  {
    NxShape * shape = shapes[nShapes];
    
    switch(shape->getType())
    {
        case NX_SHAPE_PLANE:
          //qDebug() << "NX_SHAPE_PLANE";
          drawPlane(shape->isPlane());
          break;
        case NX_SHAPE_SPHERE:
          //qDebug() << "NX_SHAPE_SPHERE";
          break;
        case NX_SHAPE_BOX:
          //qDebug() << "NX_SHAPE_BOX";
          drawBox(shape->isBox());
          break;
        case NX_SHAPE_CAPSULE:
          //qDebug() << "NX_SHAPE_CAPSULE";
          break;
        case NX_SHAPE_WHEEL:
          //qDebug() << "NX_SHAPE_WHEEL";
          break;
        case NX_SHAPE_CONVEX:
          //qDebug() << "NX_SHAPE_CONVEX";
          break;
        case NX_SHAPE_MESH:
          //qDebug() << "NX_SHAPE_MESH";
          break;
        case NX_SHAPE_HEIGHTFIELD:
          //qDebug() << "NX_SHAPE_HEIGHTFIELD";
          break;
        case NX_SHAPE_RAW_MESH:
          //qDebug() << "NX_SHAPE_RAW_MESH";
          break;
        case NX_SHAPE_COMPOUND:
          //qDebug() << "NX_SHAPE_COMPOUND";
          break;
        case NX_SHAPE_COUNT:
          //qDebug() << "NX_SHAPE_COUNT";
          break;
        case NX_SHAPE_FORCE_DWORD:
          //qDebug() << "NX_SHAPE_FORCE_DWORD";
          break;
        default:
          qDebug() << "Unknown shape!";
        break;
    } 
  } 

}

bool GLDisplay::closeTo (NxReal r, NxReal val) const {
  return abs(r - val) < 0.001;
}

void GLDisplay::drawPlane(NxPlaneShape* planeShape) const {

    NxPlane plane = planeShape->getPlane ();

    NxVec3 random = plane.normal;
    if (closeTo(random.x, 0), closeTo(random.y, 1), closeTo(random.z, 0))
      random.x += 1;
    else
      random.y += 1;
      
    NxVec3 tx = plane.normal.cross(random);
    tx.normalize ();
    //qDebug() << tx.x << ":" << tx.y << ":" << tx.z;
    NxVec3 ty = plane.normal.cross(tx);
    ty.normalize ();
    //qDebug() << ty.x << ":" << ty.y << ":" << ty.z;
    NxF32 d = plane.d;
    NxReal n = 1;
    //qDebug() << (tx.x + ty.x) + n * d << ":" << (tx.y + ty.y) + n * d << ":" << (tx.z + ty.z) + n * d;
    //qDebug() << (tx.x - ty.x) + n * d << ":" << (tx.y - ty.y) + n * d << ":" << (tx.z - ty.z) + n * d;
    //qDebug() << (-tx.x - ty.x) + n * d << ":" << (-tx.y - ty.y) + n * d << ":" << (-tx.z - ty.z) + n * d;
    //qDebug() << (-tx.x + ty.x) + n * d << ":" << (-tx.y + ty.y) + n * d << ":" << (-tx.z + ty.z) + n * d;
    
    glBegin(GL_QUADS);
      glVertex4f((tx.x + ty.x) + n * d, (tx.y + ty.y) + n * d, (tx.z + ty.z) + n * d, 0.001);
      glVertex4f((tx.x - ty.x) + n * d, (tx.y - ty.y) + n * d, (tx.z - ty.z) + n * d, 0.001);
      glVertex4f((-tx.x - ty.x) + n * d, (-tx.y - ty.y) + n * d, (-tx.z - ty.z) + n * d, 0.001);
      glVertex4f((-tx.x + ty.x) + n * d, (-tx.y + ty.y) + n * d, (-tx.z + ty.z) + n * d, 0.001);
    glEnd();
}

void GLDisplay::drawBox(NxBoxShape* boxShape) const {

  NxMat34 boxPose = boxShape->getGlobalPose ();
  
  NxF32 matrix[16];
  boxPose.getColumnMajor44(matrix);
  glPushMatrix();
  glMultMatrixf(matrix);
  
  
  glBegin(GL_QUADS);
  for (int i = 0 ; i < 2 ; i++) {
    double unit = 1*((i*2)-1);
    
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d( unit,  1,  1);
    glVertex3d( unit,  1, -1);
    glVertex3d( unit, -1, -1);
    glVertex3d( unit, -1,  1);
    
    glColor3d(0.0, 1.0, 0.0);
    glVertex3d(  1, unit,  1);
    glVertex3d(  1, unit, -1);
    glVertex3d( -1, unit, -1);
    glVertex3d( -1, unit,  1);
    
    glColor3d(0.0, 0.0, 1.0);
    glVertex3d(  1,  1, unit);
    glVertex3d(  1, -1, unit);
    glVertex3d( -1, -1, unit);
    glVertex3d( -1,  1, unit);
  }
  glEnd();

  glPopMatrix();
}

/****************************************************************************
**
** Author: Richard Baxter
**
** This function might change
**
****************************************************************************/

void GLDisplay::setColour(double red, double green, double blue, double alpha) const {

  glColor4d(red, green, blue, alpha);
}
  
  
/****************************************************************************
**
** Author: Richard Baxter
**
** This function might change
**
****************************************************************************/
void GLDisplay::drawText2D(int x, int y, const QString & str, const QFont & fnt, int listBase) const
{
  glDisplayWidget->renderText(x, y, str, fnt, listBase);
}







  
} /* end of namespace GameComponent */

} /* end of namespace ProcGen */













