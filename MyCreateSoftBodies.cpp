#include "stdafx.h"
#include "MyPhysx.h"



void World::SetupSoftBodyCube()
{
	NxSoftBodyDesc softBodyDesc;
	softBodyDesc.globalPose.t = NxVec3(0,3,0);
	softBodyDesc.particleRadius = 0.2f;
	softBodyDesc.volumeStiffness = 0.2f;
	softBodyDesc.stretchingStiffness = 0.2f;
	softBodyDesc.friction = 1.0f;
	softBodyDesc.solverIterations = 5;
	
	softBodyDesc.flags |= NX_SBF_COLLISION_TWOWAY ;

	MySoftBody *softBody = new MySoftBody(gScene, softBodyDesc, SOFTBODY_BOX,4,3,3,1);


	NxSoftBodyDesc softBodyDesc1;
	softBodyDesc1.globalPose.t = NxVec3(0,9,0);
	softBodyDesc1.particleRadius = 0.2f;
	softBodyDesc1.volumeStiffness = 0.2f;
	softBodyDesc1.stretchingStiffness = 0.2f;
	softBodyDesc1.friction = 1.0f;
	softBodyDesc1.solverIterations = 5;
	
	softBodyDesc1.flags |= NX_SBF_COLLISION_TWOWAY | NX_SBF_SELFCOLLISION;


	MySoftBody *softBody1 = new MySoftBody(gScene, softBodyDesc1, SOFTBODY_BOX,4,3,3,1);

	if (!softBody->getNxSoftBody())
	{
		printf("Error: Unable to create the SoftBody for the current scene.\n");
		delete softBody;
	}
	else
	{
		gSoftBodies.push_back(softBody);
		gSoftBodies.push_back(softBody1);
	}

	softBody = softBody1;

	if (gSoftBodies.size() > 0)
		gSelectedSoftBody = gSoftBodies[0]->getNxSoftBody();
	else
		gSelectedSoftBody = NULL;

	if (gScene->getNbActors() > 0)
		gSelectedActor = *gScene->getActors();
	else
		gSelectedActor = NULL;
	gForceStrength = 100000;
}

void World::SetupPalmScene()
{
	NxSoftBodyDesc softBodyDesc;
	softBodyDesc.globalPose.t = NxVec3(0,3,0);
	softBodyDesc.particleRadius = 0.2f; 
	softBodyDesc.volumeStiffness = 0.5;
	softBodyDesc.stretchingStiffness = 1.0f;
	softBodyDesc.friction  = 1.0f;
	softBodyDesc.flags |= NX_SBF_COLLISION_TWOWAY;

	char* fileName = "palm";

	char tetFileName[256], objFileName[256], s[256];
	//sprintf(tetFileName, "%s.tet", fileName);
	sprintf(objFileName, "%s.obj", fileName);

	ObjMesh *objMesh = new ObjMesh();
	objMesh->loadFromObjFile(FindMediaFile(objFileName, s));

	MySoftBody *softBody = new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName,s), objMesh);
	if (!softBody->getNxSoftBody())
	{
		printf("Error: Unable to create the SoftBody for the current scene.\n");
		delete softBody;
	}
	else
	{
		gSoftBodies.push_back(softBody);
		gObjMeshes.push_back(objMesh);

		softBody->getNxSoftBody()->setExternalAcceleration(NxVec3(5,0,0));

		NxActor *caps0 = CreateCompoundCapsule(NxVec3(-0.46f, -0.15f, 0.68f), 1.7f, 0.21f, 2.0f, 0.18f);
		caps0->userData = (void*)&gInvisible;
		CreateRopeSphericalJoint(NULL, caps0, NxVec3(-0.46f, -0.0f, 0.68f), NxVec3(0,0,1));

		softBody->getNxSoftBody()->attachToCollidingShapes(0);
	}
}

void World::SetupBunnyScene()
{
	NxSoftBodyDesc softBodyDesc;
	softBodyDesc.globalPose.t = NxVec3(0,35,-1);
	softBodyDesc.particleRadius = 0.2f;
	softBodyDesc.volumeStiffness = 1.0f;
	softBodyDesc.stretchingStiffness = 0.07f;
	softBodyDesc.friction = 1.0f;

	char *fileName = "bunny";	
	char tetFileName[256], objFileName[256], s[256];
	sprintf(tetFileName, "%s.tet", fileName);
	sprintf(objFileName, "%s.obj", fileName);

	ObjMesh *objMesh = new ObjMesh(); // it is for mesh surface rendering
	objMesh->loadFromObjFile(FindMediaFile(objFileName, s));  

	MySoftBody *softBody = new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName,s), objMesh);

	if (!softBody->getNxSoftBody())
	{
		printf("Error: Unable to create the SoftBody for the current scene.\n");
		delete softBody;
	} else
	{
		gSoftBodies.push_back(softBody);  // soft body is pushed to list.
		gObjMeshes.push_back(objMesh);  // surface mesh is pushed to list.
	}

	NxMat33 rot;
	NxActor *plate0 = CreateBox(NxVec3(-2.5,10,0), NxVec3(20.0f, 1.0f, 10.0f), 0,0.0f);
	rot.rotZ(1.5f);
	plate0->setGlobalOrientation(rot);
	NxActor *plate1 = CreateBox(NxVec3(2.5,10,0), NxVec3(20.0f, 1.0f, 10.0f), 0,0.0f);
	rot.rotZ(1.5f);
	plate1->setGlobalOrientation(rot);

	// set camera position and direction
	gCameraPos.set(-5.0f, 40.0f, 35.0f);
	gCameraForward.set(0.2,-1.1,-2);
	gCameraForward.normalize();
}

void World::SetupSoftWheelCarScene()
{
	NxSoftBodyDesc softBodyDesc;
	softBodyDesc.particleRadius = 0.2f;
	softBodyDesc.volumeStiffness = 1.0f;
	softBodyDesc.stretchingStiffness = 1.0f;
	softBodyDesc.friction = 1.0f;
	softBodyDesc.attachmentResponseCoefficient = 0.8f;
	softBodyDesc.tearFactor = 10.1;						// should be more than 1.0, the less, the easier to tear with flag NX_SBF_TEARABLE

	softBodyDesc.flags |= NX_SBF_HARDWARE | NX_SBF_VOLUME_CONSERVATION | NX_SBF_TEARABLE;

	char *fileName = "wheel";
	char tetFileName[256], objFileName[256], s[256];
	sprintf(tetFileName, "%s.tet", fileName);
	sprintf(objFileName, "%s.obj", fileName);

	MySoftBody *softBody;

	NxReal carHeight = 7.5f;
	NxReal stiffness = 0.9f;
	NxMat34 capsulePose = NxMat34(NxMat33(NX_IDENTITY_MATRIX), NxVec3(-4, carHeight, -5.0f));
	printf("capsulePose %f %f %f\n", capsulePose.t.x, capsulePose.t.y, capsulePose.t.z);
	capsulePose.M.rotX(NxHalfPiF32);
	NxActor *caps1 = CreateOrientedCapsule(capsulePose, 7.1f, 1.3f, 1.0f);
	capsulePose.t = NxVec3(4, carHeight, -5.0f);
	NxActor *caps2 = CreateOrientedCapsule(capsulePose, 7.1f, 1.3f, 1.0f);
	
	ObjMesh *objMesh = new ObjMesh();
	objMesh->loadFromObjFile(FindMediaFile(objFileName, s));
	
	NxMat33 rot;
	rot.rotX(NxPiF32);
	softBodyDesc.globalPose.t = NxVec3(4.0f, carHeight, 3.4f);
	softBodyDesc.globalPose.M = rot;
	softBodyDesc.stretchingStiffness = stiffness;
	softBody = new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName,s), objMesh);
	if (!softBody->getNxSoftBody())
	{
		printf("Error: Unable to create Softbody for the current scene.\n");
		delete softBody;
	}
	else
	{
		gObjMeshes.push_back(objMesh);

		// wheel 1 
		softBody->getNxSoftBody()->attachToCollidingShapes(NX_SOFTBODY_ATTACHMENT_TWOWAY);
		gSoftBodies.push_back(softBody);

		softBodyDesc.globalPose.t = NxVec3(-4.0f ,carHeight, 3.4f);
		softBodyDesc.globalPose.M = rot;
		softBodyDesc.stretchingStiffness = stiffness;
		softBody = new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName,s), objMesh);
		softBody->getNxSoftBody()->attachToCollidingShapes(NX_SOFTBODY_ATTACHMENT_TWOWAY);
		gSoftBodies.push_back(softBody);

		softBodyDesc.globalPose.t = NxVec3(4.0f, carHeight, -3.4f);
		softBodyDesc.globalPose.M.id();
		softBodyDesc.stretchingStiffness = stiffness;
		softBody = new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName,s), objMesh);
		softBody->getNxSoftBody()->attachToCollidingShapes(NX_SOFTBODY_ATTACHMENT_TWOWAY);
		gSoftBodies.push_back(softBody);

		softBodyDesc.globalPose.t = NxVec3(-4.0f, carHeight, -3.4f);
		softBodyDesc.globalPose.M.id();
		softBodyDesc.stretchingStiffness = stiffness;
		softBody = new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName,s), objMesh);
		softBody->getNxSoftBody()->attachToCollidingShapes(NX_SOFTBODY_ATTACHMENT_TWOWAY);		// bind soft wheel with capsule colliding with it
		gSoftBodies.push_back(softBody);

		NxActor *box = CreateBox(NxVec3(0,carHeight,0), NxVec3(4.6f, 0.5f, 1.0f), 0,1.0f);
		CreateRevoluteJoint(box, caps1, NxVec3(-4,carHeight,-3.5f), NxVec3(0,0,1), false);
		CreateRevoluteJoint(box, caps2, NxVec3( 4,carHeight,-3.5f), NxVec3(0,0,1), false);

	}
	gSelectedActor = caps1;

}