/*
  Bullet Continuous Collision Detection and Physics Library
  Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

  This software is provided 'as-is', without any express or implied warranty.
  In no event will the authors be held liable for any damages arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose, 
  including commercial applications, and to alter it and redistribute it freely, 
  subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/


// Modified by Peter Battaglia (2012) from HelloWorld.cpp example in Bullet SDK


/* Command line use: 

   g++ -g -I/usr/local/include/bullet -lBulletCollision -lBulletDynamics -lLinearMath -o physics_score physics_score.cpp

*/

#include <GL/glut.h>
#include "include/physics_score.h"
#include <stdio.h>
#include <time.h>





static float gtime = 0.0;
static SBullet *bullet;


static void draw(void)
{
  int j;
  float c;
  float size;
  btConvexHullShape* shape;
  //static 
  btTransform trans;
  //static 
  btScalar matrix[16];
  btCollisionObject* obj;
  btRigidBody* body;

  int nObj = bullet->dynamicsWorld->getNumCollisionObjects() - 1;


  // Clear buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  for (j=0; j<nObj; j++) {
    // Get the object
    obj = bullet->dynamicsWorld->getCollisionObjectArray()[j + 1];
    
    // Get the object's shape
    shape = (btConvexHullShape*)obj->getCollisionShape();
    

    // Get the object's state
    body = btRigidBody::upcast(obj);
    body->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(matrix);
    size = 2.0;
    
    // Draw commands
    c = (float)j / (nObj - 1);
    glColor3f(c, 0.0, 1.0 - c);
    glPushMatrix();
    glMultMatrixf(matrix);
    glutSolidCube(size);
    glPopMatrix();
  }


  // //*** draw box1 
  // glColor3f(0.0, 0.0, 1.0);
  // glPushMatrix();
  // box1->getMotionState()->getWorldTransform(trans);
  // trans.getOpenGLMatrix(matrix);
  // glMultMatrixf(matrix);
  // glutSolidCube(40);
  // glPopMatrix();

  // //*** draw box2
  // glColor3f(1.0, 1.0, 0.0);
  // glPushMatrix();
  // box2->getMotionState()->getWorldTransform(trans);
  // trans.getOpenGLMatrix(matrix);
  // glMultMatrixf(matrix);
  // glutSolidCube(10);
  // glPopMatrix();

  glutSwapBuffers();
}




static void timer(void)
{
  float dtime = gtime;
  gtime = glutGet(GLUT_ELAPSED_TIME) / 3000.0;
  dtime = gtime - dtime;

  if(bullet->dynamicsWorld) {
    bullet->dynamicsWorld->stepSimulation(dtime, 10);
  }
  glutPostRedisplay();
}




// int main(int argc, char** argv)
// {
//   //*** init Bullet Physics
//   btQuaternion qtn;

//   btCollisionShape *shape;
//   btDefaultMotionState *motionState; 

//   btDefaultCollisionConfiguration *collisionCfg 
//     = new btDefaultCollisionConfiguration();

//   btAxisSweep3 *axisSweep 
//     = new btAxisSweep3(btVector3(-100,-100,-100), btVector3(100,100,100), 128);

//   dynamicsWorld = new btDiscreteDynamicsWorld(new btCollisionDispatcher(collisionCfg), 
// 					      axisSweep, new btSequentialImpulseConstraintSolver, collisionCfg);

//   dynamicsWorld->setGravity(btVector3(0, -10, 0));


//   //*** box1 - STATIC / mass=btScalar(0.0)
//   shape = new btBoxShape(btVector3(20,20,20));

//   trans.setIdentity();
//   qtn.setEuler(0, 0.25, -0.05);
//   trans.setRotation(qtn);
//   trans.setOrigin(btVector3(0, -20, 0));
//   motionState = new btDefaultMotionState(trans);

//   box1 = new btRigidBody(btScalar(0.0), motionState, shape, btVector3(0,0,0));
//   dynamicsWorld->addRigidBody(box1);

//   //*** box2 - DYNAMIC / mass=btScalar(1.0) 
//   shape = new btBoxShape(btVector3(5,5,5));

//   trans.setIdentity();
//   qtn.setEuler(0.8, 0.7, 0.4);
//   trans.setRotation(qtn);
//   trans.setOrigin(btVector3(-10, 50, 0));
//   motionState = new btDefaultMotionState(trans);

//   box2 = new btRigidBody(btScalar(1.0), motionState, shape, btVector3(1,1,1));
//   dynamicsWorld->addRigidBody(box2);


//   //*** init GLUT 
//   glutInit(&argc, argv);
//   glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
//   glutCreateWindow("Jumpin' little BoX");

//   glutDisplayFunc(draw);
//   glutIdleFunc(timer);


//   //*** init OpenGL
//   glEnable(GL_CULL_FACE);
//   glEnable(GL_DEPTH_TEST);
//   glEnable(GL_LIGHT0);
//   glEnable(GL_LIGHTING);
//   glEnable(GL_COLOR_MATERIAL);

//   glMatrixMode(GL_PROJECTION);
//   gluPerspective( 50.0, 1.0, 20.0, 100.0);
//   glMatrixMode(GL_MODELVIEW);
//   gluLookAt(0.0, 5.0, 90.0, 0.0, 8.0, 0.0, 0.0, 1.0, 0.0); 


//   glutMainLoop();


//   //*** EXIT
//   delete shape;
//   delete motionState;
//   delete collisionCfg;
//   delete axisSweep;
// }

// ////////////////////////////////////////////////////////////////////









int main(int argc, char** argv)
{
  /////////////////////////////////////////////////////////////////////
  //
  // The first part of main() demonstrates how to set the necessary parameters

  SScene myscene;
  SSimulation mysim;
  SBullet* mybullet = new SBullet;
  double* scores;

  int nVert;

  // Some test objects
  double vertices[] = {

    // obj 1
    0., -1., 0.,
    2., -1., 0.,
    2., -1., 2.,
    0., -1., 2.,
    0., 1., 0.,
    2., 1., 0.,
    2., 1., 2.,
    0., 1., 2.,

    // obj 2
    0., 2., 0.,
    2., 2., 0.,
    2., 2., 2.,
    0., 2., 2.,
    0., 4., 0.,
    2., 4., 0.,
    2., 4., 2.,
    0., 4., 2.,

    // obj 3
    3., 4., 0.,
    5., 4., 0.,
    5., 4., 2.,
    3., 4., 2.,
    3., 6., 0.,
    5., 6., 0.,
    5., 6., 2.,
    3., 6., 2.,

    // obj 4
    3., -1., 0.,
    5., -1., 0.,
    5., -1., 2.,
    3., -1., 2.,
    3., 1., 0.,
    5., 1., 0.,
    5., 1., 2.,
    3., 1., 2.,

  };

  // Number of objects
  myscene.nObj = 4;

  // Number of vertices per object
  myscene.nVertPerObj = (int*)malloc(myscene.nObj * sizeof(int));
  myscene.nVertPerObj[0] = 8;
  myscene.nVertPerObj[1] = 8;
  myscene.nVertPerObj[2] = 8;
  myscene.nVertPerObj[3] = 8;

  // Count up vertices
  nVert = 0;
  for (int i=0; i < myscene.nObj; i++) {
    nVert += myscene.nVertPerObj[i];
  }
  // memcpy the vertex data to the myscene struct
  myscene.vertices = (double *)malloc(nVert * 3 * sizeof(double));
  memcpy(myscene.vertices, vertices, nVert * 3 * sizeof(double));

  // Simulation parameters
  mysim.stepSize = 1.0 / 60.0f;
  mysim.nRecordTimes = 5;
  mysim.recordIdx = (int*)malloc(mysim.nRecordTimes * sizeof(int));
  mysim.recordIdx[0] = 0;
  mysim.recordIdx[1] = 15;
  mysim.recordIdx[2] = 30;
  mysim.recordIdx[3] = 45;
  mysim.recordIdx[4] = 60;
  scores = (double*)malloc(myscene.nObj * mysim.nRecordTimes * 2 * sizeof(double));

  /////////////////////////////////////////////////////////////////////





  ///////////////////////////////////////////////////////////////////
  //
  // GLUT viewing setup

  //*** init GLUT 
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Test viewer");

  glutDisplayFunc(draw);
  glutIdleFunc(timer);


  //*** init OpenGL
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  glMatrixMode(GL_PROJECTION);
  gluPerspective(30.0, 1.0, 0.1, 1000.0);
  glMatrixMode(GL_MODELVIEW);
  gluLookAt(1.0, 0.0, 20.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0); 



  // Assign mybullet to the global bullet, so that glutDisplayFunc's
  // draw() can access it.
  bullet = mybullet;

  // Set up scene
  create_scene(mybullet, myscene);
  

  // Start the glut loop
  glutMainLoop();


  ///////////////////////////////////////////////////////////////////





  /////////////////////////////////////////////////////////////////////
  //

  // // Do the simulation and record the data
  // compute_speed(mybullet, mysim, scores);
  
  // Destroy scene resources
  destroy_scene(mybullet);
  /////////////////////////////////////////////////////////////////////



  /////////////////////////////////////////////////////////////////////
  //
  // Print the output, to see if it works
  //

  printf("\n-------------------------------------\n");
  printf("Results\n");
  printf("-------------------------------------\n");

  for (int k=0; k<mysim.nRecordTimes; k++) {
    printf("\nRecord step %i\n", mysim.recordIdx[k]);
    for (int j=0; j<myscene.nObj; j++) {
      int idx = k * myscene.nObj * 2 + j * 2;
      printf("Obj %i:  linvel %.3f ,  angvel= %.3f\n", j, 
	     scores[idx], scores[idx + 1]);
    }
  }
  

  
  return 0;
}
