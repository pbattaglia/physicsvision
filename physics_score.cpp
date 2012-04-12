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



#include "include/physics_score.h"


int create_scene(SBullet* bullet, SScene scene)
{
  int i, j, k;

  // Seed random
  srand(time(NULL));

  // Contains default setup for memory, collision setup.
  bullet->collisionConfiguration = new btDefaultCollisionConfiguration();

  // Default collision dispatcher.
  bullet->dispatcher = 
    new btCollisionDispatcher(bullet->collisionConfiguration);

  // btDbvtBroadphase is a good general purpose broadphase.
  bullet->overlappingPairCache = new btDbvtBroadphase();

  // Default constraint solver.
  bullet->solver = new btSequentialImpulseConstraintSolver;

  // DynamicsWorld
  bullet->dynamicsWorld = 
    new btDiscreteDynamicsWorld(bullet->dispatcher, 
				bullet->overlappingPairCache, 
				bullet->solver, 
				bullet->collisionConfiguration);
  // Turn on Earth gravity
  bullet->dynamicsWorld->setGravity(btVector3(0, -9.8, 0));


  ////////////////////////////////////////////////////////
  // Ground
  //
  // Create the ground collision geometry
  bullet->groundShape = new btBoxShape(btVector3(btScalar(50.), 
						 btScalar(50.), 
						 btScalar(50.)));
  
  // Container for all of the collision shapes
  bullet->collisionShapes.push_back(bullet->groundShape);

  // Set up ground properly
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0, -51, 0));

  btScalar mass(0.);
  
  //rigidbody is dynamic if and only if mass is non zero, otherwise
  //static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0,0,0);
  if (isDynamic) {
    bullet->groundShape->calculateLocalInertia(mass, localInertia);
  }

  //using motionstate is recommended, it provides interpolation
  //capabilities, and only synchronizes 'active' objects
  btDefaultMotionState* myMotionState = 
    new btDefaultMotionState(groundTransform);
  btRigidBody::btRigidBodyConstructionInfo 
    rbInfo(mass, myMotionState, bullet->groundShape, localInertia);
  btRigidBody* body = new btRigidBody(rbInfo);

  //add the body to the dynamics world
  bullet->dynamicsWorld->addRigidBody(body);

  ////////////////////////////////////////////////////////



  ////////////////////////////////////////////////////////
  // Objects
  //
  btConvexHullShape* convexHull;

  // Add the input points to convexHull0
  for (i=0; i<scene.nObj; i++) {
    btConvexHullShape* convexHull0 = new btConvexHullShape();

    // Add vertices to this convexHull
    for (j=0; j<scene.nVertPerObj[i]; j++) {
      int idx = i * scene.nVertPerObj[i] * 3 + j * 3;

      double* v = &scene.vertices[idx];
      double r[3];
      for (k=0; k<3; k++) {
	r[k] = 0.;//0.1 * double(rand()) / double(RAND_MAX);
      }
      btVector3 point(v[0] + r[0], v[1] + r[1], v[2] + r[2]);
      convexHull0->addPoint(point);
    }

    // //create a hull approximation (not necessary for now)
    // btShapeHull* hull = new btShapeHull(convexHull0);
    // btScalar margin = convexHull0->getMargin();
    // hull->buildHull(margin);
    // btConvexHullShape* convexHull1 = new btConvexHullShape();
    // for (j=0; j<hull->numVertices(); j++) {
    //   convexHull1->addPoint(hull->getVertexPointer()[j]); 
    // }

    // Get centerpoint of object, which we'll need to shift the objects' shapes
    btVector3 center(0., 0., 0.);
    btScalar radius(0.);
    convexHull0->getBoundingSphere(center, radius);


    // Center the objects' shapes at (0, 0, 0)
    btConvexHullShape* convexHull1 = new btConvexHullShape();
    for (j=0; j<convexHull0->getNumVertices(); j++) {
      convexHull1->addPoint(convexHull0->getUnscaledPoints()[j] - center); 
    }
    
    // Turn margin to zero
    convexHull1->setMargin(btScalar(0.02));
    
    // (just an unnecessary pointer basically)
    convexHull = convexHull1;

    // Create a dynamic rigidbody
    bullet->collisionShapes.push_back(convexHull);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);

    // rigidbody is dynamic if and only if mass is non-zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic) {
      convexHull->calculateLocalInertia(mass, localInertia);
    }

    startTransform.setOrigin(center);
		
    // Motionstate is recommended, it provides interpolation
    // capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = 
      new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo 
      rbInfo(mass, myMotionState, convexHull, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    bullet->dynamicsWorld->addRigidBody(body);

    // Get centerpoint of object, which we'll need to shift the objects' shapes
    convexHull->getBoundingSphere(center, radius);

  }
  ///////////////////////

  return 0;
}





int compute_speed(SBullet* bullet, SSimulation simParam, double* scores)
{
  int i, j, k;
  int idx;
  int nObj = bullet->dynamicsWorld->getNumCollisionObjects() - 1;

  /// Do some simulation
  for (i=0;i<simParam.recordIdx[simParam.nRecordTimes - 1] + 1;i++) {
    bullet->dynamicsWorld->stepSimulation(btScalar(simParam.stepSize), 0);

    bool f_record = false;
    for (k=0; k<simParam.nRecordTimes; k++) {
      if (i == simParam.recordIdx[k]) {
	f_record = true;
	break;
      }
    }

    // Records linear and angular velocities, if necessary
    if (f_record) {
      for (j=0; j<nObj; j++) {
	btCollisionObject* obj = 
	  bullet->dynamicsWorld->getCollisionObjectArray()[j + 1];
	btRigidBody* body = btRigidBody::upcast(obj);

	if (body && body->getMotionState()) {
	  btVector3 linvel(body->getLinearVelocity());
	  btVector3 angvel(body->getAngularVelocity());

	  idx = k * nObj * 2 + j * 2;
	  scores[idx] = double(linvel.length());
	  scores[idx + 1] = double(angvel.length());
	}
      }
    }



    // //print positions of all objects (good for debugging)
    // for (j=0; j<nObj; j++) {
    //   btCollisionObject* obj = 
    // 	bullet->dynamicsWorld->getCollisionObjectArray()[j + 1];
    //   btRigidBody* body = btRigidBody::upcast(obj);

    //   if (body && body->getMotionState()) {
    // 	btVector3 linvel(body->getLinearVelocity());
    // 	btVector3 angvel(body->getAngularVelocity());

    // 	// Printout stuff (for debugging)
    // 	if (j >= 0) {
    // 	  btTransform trans;
    // 	  body->getMotionState()->getWorldTransform(trans);

    // 	  printf("\nT %i O %i:  ", i, j);
    // 	  printf("X %.3f, %.3f, %.3f\t", 
    // 	    	 float(trans.getOrigin().getX()), 
    // 	    	 float(trans.getOrigin().getY()), 
    // 	    	 float(trans.getOrigin().getZ()));


    // 	  printf("vL %.3f, %.3f, %.3f |%.3f|\t", 
    // 		 float(linvel.x()), 
    // 		 float(linvel.y()),
    // 		 float(linvel.z()),
    // 		 float(linvel.length()));

    // 	  // printf("aV %.3f, %.3f, %.3f |%.3f|\t", 
    // 	  // 	 float(angvel.x()), 
    // 	  // 	 float(angvel.y()),
    // 	  // 	 float(angvel.z()),
    // 	  // 	 float(angvel.length())
    // 	  // 	 );
    // 	}     
    //   }
    // }




  }
  return 0;
}


int destroy_scene(SBullet* bullet)
{
  int i, j;

  // cleanup in the reverse order of creation/initialization

  // remove the rigidbodies from the dynamics world and delete them
  for (i=bullet->dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
    btCollisionObject* obj = 
      bullet->dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    bullet->dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  // delete collision shapes
  for (j=0;j<bullet->collisionShapes.size();j++) {
    btCollisionShape* shape = bullet->collisionShapes[j];
    bullet->collisionShapes[j] = 0;
    delete shape;
  }

  // delete dynamics world
  delete bullet->dynamicsWorld;

  // delete solver
  delete bullet->solver;

  //delete broadphase
  delete bullet->overlappingPairCache;

  //delete dispatcher
  delete bullet->dispatcher;

  delete bullet->collisionConfiguration;

  //next line is optional: it will be cleared by the destructor when
  //the array goes out of scope
  bullet->collisionShapes.clear();

  return 0;
}
