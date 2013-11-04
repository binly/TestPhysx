#include "stdafx.h"
#include "MyPhysx.h"
#include "UserData.h"

void testSphericalJoint(World* world)
{
	NxActor* capsule1 = world->MyCreateCapsule(NxVec3(0,5,0), 3, 0.5, 10);
	capsule1->raiseBodyFlag(NX_BF_KINEMATIC);
	NxActor* capsule2 = world->MyCreateCapsule(NxVec3(0,1,0), 3, 0.5, 10);
	capsule2->setLinearDamping(0.5);

	NxVec3 globalAnchor = NxVec3(0, 5, 0);
	NxVec3 globalAxis = NxVec3(0,1,0);
	world->CreateSphericalJoint(capsule1, capsule2, globalAnchor, globalAxis);


	world->gSelectedActor = capsule2;
	world->gForceStrength = 75000;
	world->bPause = true;
}


void testPrismaticJoints(World* world)
{
	NxActor* box1 = world->CreateBox(NxVec3(0,5,0), NxVec3(1,2,1), 0,10);
	box1->raiseBodyFlag(NX_BF_KINEMATIC);
	NxActor* box2 = world->CreateBox(NxVec3(0,3,0), NxVec3(0.5,2,0.5),0, 10);
	box2->setLinearDamping(0.5);

	NxVec3 globalAnchor = NxVec3(0,7,0);
	NxVec3 globalAxis = NxVec3(0,1,0);
	world->CreatePrismaticJoint(box1, box2, globalAnchor, globalAxis);

	world->gSelectedActor = box2;
	world->gForceStrength = 40000;
}

void testCylindricalJoint(World* world)
{
	NxActor* capsule1 = world->MyCreateCapsule(NxVec3(0,5,0), 4, 0.85, 10);
	NxActor* capsule2 = world->MyCreateCapsule(NxVec3(0,3,0), 3, 0.5, 10);
	capsule1->raiseBodyFlag(NX_BF_KINEMATIC);
	capsule2->setLinearDamping(0.5);
	capsule2->setAngularDamping(0.5);

	NxVec3 globalAnchor = NxVec3(0,7,0);
	NxVec3 globalAxis = NxVec3(0,1,0);
	world->CreateCylindricalJoint(capsule1, capsule2, globalAnchor, globalAxis);

	world->gSelectedActor = capsule2;
	world->gForceStrength = 40000;
	world->bPause = true;
}

void testPointOnLineJoint(World* world)
{
	NxActor* wedge = world->CreateDownWedge(NxVec3(0,6,0), NxVec3(5,1,1), 10);
	if (wedge == NULL)
		printf("wedge is null\n");
	wedge->raiseBodyFlag(NX_BF_KINEMATIC);
	
	NxActor* pyramid = world->CreateRegularPyramid(NxVec3(0,4,0), NxVec3(1,1,1), 10);
	pyramid->setLinearDamping(0.5);
	pyramid->setAngularDamping(0.5);

	NxVec3 globalAnchor = NxVec3(0,6,0);
	NxVec3 globalAxis = NxVec3(1,0,0);

	
	NxPointOnLineJoint* polJoint = world->CreatePointOnLineJoint(wedge, pyramid, globalAnchor, globalAxis);
	polJoint->setLimitPoint(globalAnchor);
	// Add left-right limiting planes
	polJoint->addLimitPlane(-globalAxis, globalAnchor + 5*globalAxis);
	polJoint->addLimitPlane(globalAxis, globalAnchor - 5*globalAxis);
	

	AddUserDataToActors(world->gScene);

	world->gSelectedActor = pyramid;
	//world->gForceStrength = 25000;
}

void testPointInPlaneJoint(World* world)
{
	NxActor* box = world->CreateBox(NxVec3(0,6,0), NxVec3(5,0.5,5), 0,10);
	box->raiseBodyFlag(NX_BF_KINEMATIC);

	NxActor* pyramid = world->CreateRegularPyramid(NxVec3(0,4,0), NxVec3(1,1,1), 10);
	pyramid->setLinearDamping(0.5);
	pyramid->setAngularDamping(0.5);

	NxVec3 globalAnchor = NxVec3(0,6,0);
	NxVec3 globalAxis = NxVec3(0,1,0);
	
	NxPointInPlaneJoint* pipJoint = world->CreatePointInPlaneJoint(box, pyramid, globalAnchor, globalAxis);
	pipJoint->setLimitPoint(globalAnchor);

	pipJoint->addLimitPlane(NxVec3(1,0,0), globalAnchor - 5*NxVec3(1,0,0));		// right plane
	pipJoint->addLimitPlane(NxVec3(-1,0,0), globalAnchor + 5*NxVec3(1,0,0));	// left plane	
	pipJoint->addLimitPlane(NxVec3(0,0,1), globalAnchor - 5*NxVec3(0,0,1));		// behind plane
	pipJoint->addLimitPlane(NxVec3(0,0,-1), globalAnchor + 5*NxVec3(0,0,1));	// forward plane

	AddUserDataToActors(world->gScene);
	world->gSelectedActor = pyramid;
	world->gForceStrength = 15000;
}

void testPulleyJoint(World* world)
{
	NxActor* capsule1 = world->MyCreateCapsule(NxVec3(-1,4,0), 1, 0.5, 10);
	NxActor* capsule2 = world->MyCreateCapsule(NxVec3(1,4,0), 1, 0.5, 10);

	// Motor specs
	NxMotorDesc gMotorDesc;
	gMotorDesc.maxForce = NX_MAX_REAL;
	gMotorDesc.freeSpin = false;
	gMotorDesc.velTarget = 0;

	// Create pulley joint
	NxVec3 pulley1 = NxVec3(-1,8,0);
	NxVec3 pulley2 = NxVec3(1,8,0);
	NxVec3 globalAxis = NxVec3(0,-1,0);
	NxPulleyJoint* pulleyJoint = world->CreatePulleyJoint(capsule1, capsule2, pulley1, pulley2, globalAxis, 4, 1, gMotorDesc);

	world->gSelectedActor = capsule1;
	world->gForceStrength = 15000;
}

void testDistanceJoint(World* world)
{
	NxActor* sphere1 = world->CreateSphere(NxVec3(-3,4,0), 1, 5);
	sphere1->raiseBodyFlag(NX_BF_FROZEN_ROT);
	NxActor* sphere2 = world->CreateSphere(NxVec3(3,4,0), 1, 5);
	sphere2->raiseBodyFlag(NX_BF_FROZEN_ROT);

	// Create distance joint
	NxVec3 anchor1 = NxVec3(0,1,0);
	NxVec3 anchor2 = NxVec3(0,1,0);
	NxVec3 globalAxis2 = NxVec3(-1,0,0);
	NxDistanceJoint* distanceJoint = world->CreateDistanceJoint(sphere1, sphere2, anchor1, anchor2, globalAxis2);

	world->gSelectedActor = sphere1;
	world->gForceStrength = 35000;
}