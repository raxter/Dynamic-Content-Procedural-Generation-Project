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
  glOrtho(	0, width/10, 0, height/10, -100, 100);
  //gluPerspective( 75/*GLdouble	fovy*/,
	//		            1/*GLdouble	aspect*/,
	//		            0.1/*GLdouble	zNear*/,
	//		            10000/*GLdouble	zFar*/ );

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
  qDebug() << "GLDisplay::initRenderStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::cleanupRenderStep()
{
  qDebug() << "GLDisplay::cleanupRenderStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}



/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
unsigned int  GLDisplay::bindTexture(const QImage& image) const {
  //qDebug() << "GLDisplay::bindTexture";
  //qDebug() << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  return -1;//glDisplayWidget->bindTexture(image);
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::drawBody(b2Body* body) const {
  //qDebug() << "GLDisplay::drawBody";
  //qDebug() << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  b2Shape *shape = body->GetShapeList ();
  
  while (shape) {
    
    glPushMatrix();
    
    glTranslatef(body->GetPosition ().x, body->GetPosition ().y, 0);
    
    glRotatef(body->GetAngle()*180/M_PI,0,0,1);
      
    glBegin(GL_LINE_STRIP);
    //polygon
    if (shape->GetType ()) {
      b2PolygonShape* poly = (b2PolygonShape*)shape;
      const b2Vec2* vertexArray = poly->GetVertices ();
      for (int i = 0 ; i < poly->GetVertexCount() ; i++)
        glVertex2f(vertexArray[i].x, vertexArray[i].y);
    }
    //circle
    else {
      b2CircleShape* circ = (b2CircleShape*)shape;
      for (int i = 0 ; i < 17 ; i++)
        glVertex2f(circ->GetRadius ()*sin(M_PI*2*i/16), circ->GetRadius ()*cos(M_PI*2*i/16));
      glVertex2f(circ->GetRadius ()*sin(M_PI*2*8/16), circ->GetRadius ()*cos(M_PI*2*8/16));
    }
    glEnd();
    
    glPopMatrix();
      
    
    shape = shape->GetNext ();
  }
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::drawCube(double cx, double cy, double cz, double sx, double sy, double sz) const
{
  glPushMatrix();
  
  //glScaled(100,100,100);
  glBegin(GL_QUADS);
  for (int i = 0 ; i < 2 ; i++) {
    double unit = 0.5*((i*2)-1);
    
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d( unit,  0.5,  0.5);
    glVertex3d( unit,  0.5, -0.5);
    glVertex3d( unit, -0.5, -0.5);
    glVertex3d( unit, -0.5,  0.5);
    
    glColor3d(0.0, 1.0, 0.0);
    glVertex3d(  0.5, unit,  0.5);
    glVertex3d(  0.5, unit, -0.5);
    glVertex3d( -0.5, unit, -0.5);
    glVertex3d( -0.5, unit,  0.5);
    
    glColor3d(0.0, 0.0, 1.0);
    glVertex3d(  0.5,  0.5, unit);
    glVertex3d(  0.5, -0.5, unit);
    glVertex3d( -0.5, -0.5, unit);
    glVertex3d( -0.5,  0.5, unit);
  }
  glEnd();
  
  glPopMatrix();
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::drawPolygon(const QVector<double>& points) const
{
  //qDebug() << "currentContext: " << QGLContext::currentContext ();
  //qDebug() << "drawPolygon - " << points;
  glBegin(GL_POLYGON);
    for (int i = 0 ; i < points.size() ; i+= 3) {
      //qDebug() << points[i];
      //qDebug() << points[i+1];
      //qDebug() << points[i+2];
      //qDebug() << "---";
        
      glVertex3d(points[i], points[i+1], points[i+2]);
    }  
  glEnd();
  
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













