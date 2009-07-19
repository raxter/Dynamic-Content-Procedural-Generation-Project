#include "test_game.h"

//#include "abstract_game_components/game.h"
//#include "abstract_game_components/display.h"

#include <QDebug>


#include <cmath>

namespace ProcGen {

namespace GameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
TestGame::TestGame() : AbstractGameComponent::Game(), framecount(0)
{
  directionOffset [0] = 0;
  directionOffset [Fore|Back] = 0;
  directionOffset [Left|Rght] = 0;
  directionOffset [Fore|Back|Left|Rght] = 0;
  
  directionOffset [Fore] = 0;
  directionOffset [Back] = 2*M_PI*2/4;
  directionOffset [Left] = 2*M_PI*3/4;
  directionOffset [Rght] = 2*M_PI*1/4;
  
  directionOffset [Fore|Left] = 2*M_PI*7/8;
  directionOffset [Fore|Rght] = 2*M_PI*1/8;
  directionOffset [Back|Left] = 2*M_PI*5/8;
  directionOffset [Back|Rght] = 2*M_PI*3/8;
  
  
  directionOffset [Fore|Left|Rght] = directionOffset [Fore];
  directionOffset [Back|Left|Rght] = directionOffset [Back];
  directionOffset [Left|Fore|Back] = directionOffset [Left];
  directionOffset [Rght|Fore|Back] = directionOffset [Rght];
  
  offx = 50;
  offy = 10;
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
TestGame::~TestGame()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::resizeStep(int width, int height) {
  display_width = width;
  
  display_height = height;
  
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::initStep(const AbstractGameComponent::Display& displayer)
{
  qDebug() << "TestGame::initStep";
  qDebug() << "currentContext: " << QGLContext::currentContext ();
  
  launching = 0;
  spin_force = 0;
  
// Define the size of the world. Simulation will still work
	// if bodies reach the end of the world, but it will be slower.
	worldAABB = new b2AABB();
	worldAABB->lowerBound.Set(-1500.0f, -1000.0f);
	worldAABB->upperBound.Set( 1500.0f,  1000.0f);
 
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);
 
	// Do we want to let bodies sleep?
	bool doSleep = true;
 
	// Construct a world object, which will hold and simulate the rigid bodies.
	world = new b2World(*worldAABB, gravity, doSleep);
 
 
	createBox(0, -10, 2100, 10, 0, 0);
	for (int i = 0 ; i < 10 ; i++)
	  createBox(i, 20+i*2.5, 1, 1, 1);
	  
	ball[0] = createCircle(1000, 10, 5, 1, 1);
	ball[0]->SetBullet (true);
	ball[1] = createCircle(-1000, 10, 5, 1, 1);
	ball[1]->SetBullet (true);
	
  backgroundTexture = QImage("../content/background.png");
  backgroundTextureId = displayer.bindTexture(backgroundTexture);
  
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/

b2Body* TestGame::createCircle(float32 x, float32 y, float32 radius, float32 angle, float32 density) {

  // Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.position.Set(x, y);
	bodyDef.angle = angle;
	b2Body* body = world->CreateBody(&bodyDef);
 
	// Define another box shape for our dynamic body.
	b2CircleDef shapeDef;
	shapeDef.radius = radius;
 
	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = density;
 
	// Override the default friction.
	shapeDef.friction = 0.3f;
 
	// Add the shape to the body.
	body->CreateShape(&shapeDef);
 
	// Now tell the dynamic body to compute it's mass properties base
	// on its shape.
	body->SetMassFromShapes();
  bodies.append(body);
  
  return body;
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/

b2Body* TestGame::createBox(float32 x, float32 y, float32 width, float32 height, float32 angle, float32 density) {

  // Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.position.Set(x, y);
	bodyDef.angle = angle;
	b2Body* body = world->CreateBody(&bodyDef);
 
	// Define another box shape for our dynamic body.
	b2PolygonDef shapeDef;
	shapeDef.SetAsBox(width, height);
 
	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = density;
 
	// Override the default friction.
	shapeDef.friction = 0.3f;
 
	// Add the shape to the body.
	body->CreateShape(&shapeDef);
 
	// Now tell the dynamic body to compute it's mass properties base
	// on its shape.
	body->SetMassFromShapes();
  bodies.append(body);
  
  return body;
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::logicStep(const AbstractGameComponent::ControlInterface& controlInterface)
{
  qDebug() << "TestGame::logicStep";
  qDebug() << "currentContext: " << QGLContext::currentContext ();
  framecount++;
  
  mousePos = controlInterface.getMousePosition();
  mouseMove = controlInterface.getMouseMovement();
  
  b2Vec2 diff =  (ball[0]->GetPosition() - ball[1]->GetPosition());
  ball_dist = diff.Length();
    
  if (controlInterface.isMouseDown(Qt::LeftButton)) {
    offy -= 0.03*mouseMove.y();
    offx += 0.03*mouseMove.x();
  }
  
	if (controlInterface.isKeyDown(Qt::Key_D))
	  spin_force -= 100;
	if (controlInterface.isKeyDown(Qt::Key_A))
	  spin_force += 100;
	if (controlInterface.isKeyDown(Qt::Key_Space))
	  spin_force = 0;
	  
  //TODO make based on ball[0]->spin
	
  b2Vec2 forward = ball[0]->GetLinearVelocity();
  forward.Normalize ();
  
  float theta = M_PI/2;
  b2Vec2 spin_dir;
  spin_dir.x = cos(theta)*forward.x - sin(theta)*forward.y;
  spin_dir.y = sin(theta)*forward.x + cos(theta)*forward.y;
  
  spin_dir*=spin_force;
  
  ball[0]->ApplyForce(spin_dir, b2Vec2());
  
	if (controlInterface.isKeyTapped(Qt::Key_J))
	  launching = 300;
	
	
	//bool force_ang_vel = false;
	if (launching > 10) {
	  ball[0]->ApplyForce(b2Vec2(-2500.0,  2500.0), b2Vec2());
	  ball[1]->ApplyForce(b2Vec2( 2500.0,  2500.0), b2Vec2());
	}
	if (launching > 0) {
	  launching--;
	}
	ball[0]->SetAngularVelocity(0);
	ball[1]->SetAngularVelocity(0);
  
  // Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
  
  float slowdown_affect = 100.0/ball_dist;
  
	float32 timeStep = 1.0f / 60.0f;
	if (slowdown_affect*slowdown_affect > 16)
	  timeStep /= 16;
	else if (slowdown_affect > 1.0)
	  timeStep /= slowdown_affect*slowdown_affect;
	/*if (controlInterface.isKeyDown(Qt::Key_H)) {
	  timeStep /= 8;
	}*/
	int32 velocityIterations = 8;
	int32 positionIterations = 1;
	
 
  calculateOffsetAndZoom();
 
	world->Step(timeStep, velocityIterations);
}

void TestGame::calculateOffsetAndZoom() {

  b2Vec2 avg = (ball[0]->GetPosition() + ball[1]->GetPosition());
  offx = avg.x/2;
  offy = avg.y/2;
  
  float desired_dist = abs(ball[0]->GetPosition().x - ball[1]->GetPosition().x);
  scale_zoom = display_width/fmax(200, desired_dist+30);
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::renderStep(const AbstractGameComponent::Display& displayer)
{


  qDebug() << "TestGame::renderStep";
  //qDebug() << displayer;
  qDebug() << "currentContext: " << QGLContext::currentContext ();

  //TODO there should be NO gl in this class
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glScaled(scale_zoom, scale_zoom, 0);
  glTranslated(-offx, -offy, 0);
  glTranslated((0.5*display_width)/scale_zoom,(0.5*display_height)/scale_zoom,0);
  
  //qDebug() << "bind? " << glIsEnabled(GL_TEXTURE_2D);
  //qDebug() << QDir(".").absolutePath ();
  //qDebug() << cloudImage.width();
  
  
  glBindTexture(GL_TEXTURE_2D, backgroundTextureId);
  
  glColor3d(1.0,1.0,1.0);
  
  float size = 1000;
  glBegin(GL_QUADS);
  for (int i = -1 ; i <= 0 ; i++) {
    for (int j = -0 ; j <= 0 ; j++) {
      glTexCoord2f(1,0); glVertex3f(size*(i),   size*(j),   1);
      glTexCoord2f(1,1); glVertex3f(size*(i),   size*(j+1), 1);
      glTexCoord2f(0,1); glVertex3f(size*(i+1), size*(j+1), 1);
      glTexCoord2f(0,0); glVertex3f(size*(i+1), size*(j),   1);
    }
  }
  glEnd();
  glBindTexture(GL_TEXTURE_2D, -1);
  
  
  Q_FOREACH(b2Body * body, bodies) {
    displayer.drawBody(body);
  }
  
 
	
  glColor3d(1.0,1.0,1.0);
  displayer.drawText2D(10,10, QString("Pos ") + QString::number(offx) +", "+QString::number(offy), QFont());
  displayer.drawText2D(10,30, QString("Launching = ") + QString::number(launching), QFont());
  displayer.drawText2D(10,50, QString("Distance = ") + QString::number(ball_dist), QFont());
  displayer.drawText2D(10,70, QString("Distance^-1 = ") + QString::number(1.0/ball_dist*1000), QFont());
  displayer.drawText2D(10,90, QString("Ang Vel = ") + QString::number(ball[0]->GetAngularVelocity()), QFont());
  displayer.drawText2D(10,110, QString("spin_force = ") + QString::number(spin_force), QFont());
}

void TestGame::cleanUpStep() {
}

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */

