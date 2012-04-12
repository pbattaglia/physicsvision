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


*/


#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include <stdio.h>
#include <time.h>



typedef struct
{
  int nObj;
  int* nVertPerObj;
  double* vertices;
} SScene;


typedef struct
{
  double stepSize;
  int nRecordTimes;
  int* recordIdx;
} SSimulation;


typedef struct
{
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* overlappingPairCache;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;
  btCollisionShape* groundShape;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;
} SBullet;


int create_scene(SBullet* bullet, SScene scene);

int compute_speed(SBullet* bullet, SSimulation simParam, double* scores);

int destroy_scene(SBullet* bullet);
