#include "stdafx.h"
#include "MyPhysx.h"

#include <fstream>

#define RAD3 1.7320508076f

using namespace std;

NxArray<NxActor*, NxAllocatorDefault> World::gMetalCores;

void World::SetupCurtainScene()
{
	NxActor* sphere1 = CreateSphere(NxVec3(-1,0,-0.5), 1, 10);
	NxActor* box1 = CreateBox(NxVec3(1,0,-1), NxVec3(1,1,1), 0,10);

	// Create the box that the cloth will attach to
	NxActor* box2 = CreateBox(NxVec3(0,6.5,0), NxVec3(5,0.5,0.5), 0,10);
	box2->setLinearDamping(0.5);

	// Limit the movement to the box by a joint
	NxD6JointDesc d6Desc;
	d6Desc.actor[0] = NULL;
	d6Desc.actor[1] = box2;
	NxVec3 globalAnchor(0,7,0);
	d6Desc.localAnchor[0] = globalAnchor;
	box2->getGlobalPose().multiplyByInverseRT(globalAnchor, d6Desc.localAnchor[1]);

	d6Desc.localAxis[0]   = NxVec3(1,0,0);
	d6Desc.localNormal[0] = NxVec3(0,1,0);
	d6Desc.localAxis[1]   = NxVec3(1,0,0);
	d6Desc.localNormal[1] = NxVec3(0,1,0);
	d6Desc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.xMotion = NX_D6JOINT_MOTION_FREE;
	d6Desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.zMotion = NX_D6JOINT_MOTION_LOCKED;

	NxJoint* d6Joint = gScene->createJoint(d6Desc);

	// Create cloth
	NxClothDesc clothDesc;
	clothDesc.globalPose.t = NxVec3(4,7,0);
	clothDesc.thickness = 0.2;
	clothDesc.bendingStiffness = 0.5;
	clothDesc.flags |= NX_CLF_BENDING;
	clothDesc.flags |= NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION;

	//clothDesc.flags |= NX_CLF_HARDWARE;

	MyCloth *regularCloth = new MyCloth(gScene, clothDesc, 8, 7, 0.15, "nvidia.bmp");

	if (!regularCloth->getNxCloth())
	{
		printf("Error: Cloth creation failed.\n");
		delete regularCloth;

	}
	else
	{
		gCloths.push_back(regularCloth);
		// Hang up the cloth
		regularCloth->getNxCloth()->attachToShape(*box2->getShapes(), NX_CLOTH_ATTACHMENT_TWOWAY);
	}
	gSelectedActor = box2;
}

void World::SetupFlagScene()
{
	NxActor* capsule = MyCreateCapsule(NxVec3(0.0f,-0.2f,0.0f), 10, 0.2f, 0);

	// Create Cloth
	NxClothDesc clothDesc;
	clothDesc.globalPose.t = NxVec3(0,10,0);
	clothDesc.globalPose.M.rotX(-NxHalfPiF32);
	clothDesc.thickness = 0.2;
	clothDesc.bendingStiffness = 0.5;
	clothDesc.dampingCoefficient = 1;
	clothDesc.flags |= NX_CLF_BENDING;
	//clothDesc.flags |= NX_CLF_DAMPING | NX_CLF_COMDAMPING;
	clothDesc.flags |= NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION;
	clothDesc.windAcceleration = NxVec3(-20, 20, 0);	// set wind

	MyCloth* regularCloth = new MyCloth(gScene, clothDesc, 5, 5, 0.15, "nvidia.bmp");

	if (!regularCloth->getNxCloth())
	{
		printf("Error: Cloth creation failed.\n");
		delete regularCloth;
	}
	else
	{
		gCloths.push_back(regularCloth);
		regularCloth->getNxCloth()->attachToShape(*capsule->getShapes(), NX_CLOTH_ATTACHMENT_TWOWAY);
	}
}

void GetPos(NxVec3 &maxPos, NxVec3 &minPos, NxCloth *cloth)
{
	NxVec3 vetex[2000];
	ofstream outFile("vetex");
	NxReal maxX = -999.0f, minX = 999.0f;
	NxReal maxY = -999.0f, minY = 999.0f;
	NxReal maxZ = -999.0f, minZ = 999.0f;

	memset(vetex, 0, 2000 * sizeof(NxVec3));
	cloth->getPositions(vetex);
	for (int i = 0; i < 2000; i++)
	{
		if (vetex[i].x == 0 && vetex[i].y == 0 || vetex[i].z == 0)              
		{
			//printf("i = %d\n", i); 
			break;
		}
		else
		{
			if (vetex[i].x > maxX)	maxX = vetex[i].x;
			if (vetex[i].y > maxY)	maxY = vetex[i].y;
			if (vetex[i].z > maxZ)	maxZ = vetex[i].z;

			if (vetex[i].x < minX)	minX = vetex[i].x;
			if (vetex[i].y < minY)	minY = vetex[i].y;
			if (vetex[i].z < minZ)	minZ = vetex[i].z;
			//printf("vetex %f %f %f\n", vetex[i].x, vetex[i].y, vetex[i].z);
			//outFile << vetex[i].x << "\t" << vetex[i].y << "\t" << vetex[i].z << endl;
		}
	}
	minPos.setx(minX);
	minPos.sety(minY);
	minPos.setz(minZ);

	maxPos.setx(maxX);
	maxPos.sety(maxY);
	maxPos.setz(maxZ);

	outFile << maxX << endl;
	outFile.close();
}


void World::CreateSoftComb()
{
	NxReal combLength = 4.1709f, combWidth = 4.8161f, combHeight = 5.9620f;
	NxReal x_offset, z_offset = 0.0f;

	for (int i = 0; i < 3; i++)
	{
		NxClothDesc clothDesc;
		clothDesc.globalPose.t = NxVec3(-2.5 - i*combLength - 0.05, 0.1, 0);
		
		clothDesc.thickness = 0.01f;
		clothDesc.pressure = 1.0f;
		clothDesc.stretchingStiffness = 1.0f;
		clothDesc.bendingStiffness = 1.0f;
		clothDesc.friction = 0.1;
		clothDesc.collisionResponseCoefficient = 0.1f;
		clothDesc.globalPose.M.rotY(-NxPi/6);

		clothDesc.density = 0.05f;
		clothDesc.flags |= NX_CLF_PRESSURE | NX_CLF_DAMPING;
		clothDesc.flags |= NX_CLF_BENDING | NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION | NX_CLF_TRIANGLE_COLLISION;

		MyCloth* objCloth = new MyCloth(gScene, clothDesc, "data/newmesh.obj", 0.3);
		NxCloth* cloth = objCloth->getNxCloth();

		NxVec3 maxPos, minPos;
		GetPos(maxPos, minPos, cloth);
		x_offset = (maxPos.x - minPos.x) / 4;
		z_offset = (maxPos.z - minPos.z) / 4;
		printf("x_offset is %f\n", x_offset);

		NxVec3 pos1(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2);	//mid-up   (fb ud)
		NxVec3 pos2(minPos.x, minPos.y, (maxPos.z + minPos.z)/2);	//mid-bottom
		NxVec3 pos3(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2);// mid-mid

		NxVec3 pos4(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4); //back-up
		NxVec3 pos5(minPos.x, minPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4); //back-bottom
		NxVec3 pos6(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4); //back-mid

		NxVec3 pos7(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4); //forward-up
		NxVec3 pos8(minPos.x, minPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4); //forward-bottom
		NxVec3 pos9(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);	//forward-mid

		// new pos
		NxVec3 pos11(minPos.x + x_offset, maxPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4 + z_offset/2);
		NxVec3 pos22(minPos.x + x_offset, minPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4 + z_offset/2);
		NxVec3 pos33(minPos.x + x_offset, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4 + z_offset/2);

		NxReal adhereX1 = 0.05f, adhereX2 = 0.05f, radius = 0.05f;
		NxActor* adhere1 = MyCreateCapsule(pos1, adhereX1, adhereX1, 1);
		NxActor* adhere2 = MyCreateCapsule(pos2, adhereX1, adhereX1, 1);
		NxActor* adhere3 = MyCreateCapsule(pos3, adhereX2, adhereX1, 1);

		NxActor* adhere4 = MyCreateCapsule(pos4, adhereX2, adhereX1, 1);
		NxActor* adhere5 = MyCreateCapsule(pos5, adhereX2, adhereX1, 1);
		NxActor* adhere6 = MyCreateCapsule(pos6, adhereX2, adhereX1, 1);

		NxActor* adhere7 = MyCreateCapsule(pos7, adhereX2, adhereX1, 1);
		NxActor* adhere8 = MyCreateCapsule(pos8, adhereX2, adhereX1, 1);
		NxActor* adhere9 = MyCreateCapsule(pos9, adhereX2, adhereX1, 1);

		NxActor* adhere10 = CreateSphere(pos11, radius, 1);
		NxActor* adhere11 = CreateSphere(pos22, radius, 1);
		NxActor* adhere12 = CreateSphere(pos33, radius, 1);

		adhere10->putToSleep();
		adhere11->putToSleep();
		adhere12->putToSleep();

		NxQuat quat(90.0f, NxVec3(0,0,1));
		NxMat33 m;
		m.id();
		m.fromQuat(quat);
		NxMat34 m34;
		m34.M = m;
		m34.t = adhere1->getGlobalPosition();
		adhere1->setGlobalPose(m34);
		m34.t = adhere2->getGlobalPosition();
		adhere2->setGlobalPose(m34);
		m34.t = adhere3->getGlobalPosition();
		adhere3->setGlobalPose(m34);
		m34.t = adhere4->getGlobalPosition();
		adhere4->setGlobalPose(m34);
		m34.t = adhere5->getGlobalPosition();
		adhere5->setGlobalPose(m34);
		m34.t = adhere6->getGlobalPosition();
		adhere6->setGlobalPose(m34);
		m34.t = adhere7->getGlobalPosition();
		adhere7->setGlobalPose(m34);
		m34.t = adhere8->getGlobalPosition();
		adhere8->setGlobalPose(m34);
		m34.t = adhere9->getGlobalPosition();
		adhere9->setGlobalPose(m34);

		cloth->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);
		gSelectedCloth = cloth;
		cloth->putToSleep();
		gCloths.push_back(objCloth);
	}

	for (int i = 0; i < 3; i++)
	{
		NxClothDesc clothDesc;
		clothDesc.globalPose.t = NxVec3(-2.5 - i*combLength - combLength/2 - 0.05, 0.1, 3*combWidth/4);
	
		clothDesc.thickness = 0.01f;
		clothDesc.pressure = 1.0f;
		clothDesc.stretchingStiffness = 1.0f;
		clothDesc.bendingStiffness = 1.0f;
		clothDesc.friction = 0.1;
		clothDesc.globalPose.M.rotY(-NxPi/6);
		clothDesc.collisionResponseCoefficient = 0.1f;

		clothDesc.density = 0.05f;
		clothDesc.flags |= NX_CLF_PRESSURE | NX_CLF_DAMPING;
		clothDesc.flags |= NX_CLF_BENDING | NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION | NX_CLF_TRIANGLE_COLLISION;

		MyCloth* objCloth = new MyCloth(gScene, clothDesc, "data/newmesh.obj", 0.3);
		NxCloth* cloth = objCloth->getNxCloth();

		NxVec3 maxPos, minPos;
		GetPos(maxPos, minPos, cloth);

		NxVec3 pos1(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2);
		NxVec3 pos2(minPos.x, minPos.y, (maxPos.z + minPos.z)/2);
		NxVec3 pos3(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2);

		NxVec3 pos4(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4);
		NxVec3 pos5(minPos.x, minPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4);
		NxVec3 pos6(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4);

		NxVec3 pos7(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);
		NxVec3 pos8(minPos.x, minPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);
		NxVec3 pos9 (minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);

		// new pos
		NxVec3 pos11(minPos.x + x_offset, maxPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4 - z_offset/2);
		NxVec3 pos22(minPos.x + x_offset, minPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4 - z_offset/2);
		NxVec3 pos33(minPos.x + x_offset, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4 - z_offset/2);

		NxReal adhereX1 = 0.05f, adhereX2 = 0.05f, radius = 0.05f;
		NxActor* adhere1 = MyCreateCapsule(pos1, adhereX1, adhereX1, 1);
		NxActor* adhere2 = MyCreateCapsule(pos2, adhereX1, adhereX1, 1);
		NxActor* adhere3 = MyCreateCapsule(pos3, adhereX2, adhereX1, 1);

		NxActor* adhere4 = MyCreateCapsule(pos4, adhereX2, adhereX1, 1);
		NxActor* adhere5 = MyCreateCapsule(pos5, adhereX2, adhereX1, 1);
		NxActor* adhere6 = MyCreateCapsule(pos6, adhereX2, adhereX1, 1);

		NxActor* adhere7 = MyCreateCapsule(pos7, adhereX2, adhereX1, 1);
		NxActor* adhere8 = MyCreateCapsule(pos8, adhereX2, adhereX1, 1);
		NxActor* adhere9 = MyCreateCapsule(pos9, adhereX2, adhereX1, 1);

		NxActor* adhere10 = CreateSphere(pos11, radius, 1);
		NxActor* adhere11 = CreateSphere(pos22, radius, 1);
		NxActor* adhere12 = CreateSphere(pos33, radius, 1);

		NxQuat quat(90.0f, NxVec3(0,0,1));
		NxMat33 m;
		m.id();
		m.fromQuat(quat);
		NxMat34 m34;
		m34.M = m;
		m34.t = adhere1->getGlobalPosition();
		adhere1->setGlobalPose(m34);
		m34.t = adhere2->getGlobalPosition();
		adhere2->setGlobalPose(m34);
		m34.t = adhere3->getGlobalPosition();
		adhere3->setGlobalPose(m34);
		m34.t = adhere4->getGlobalPosition();
		adhere4->setGlobalPose(m34);
		m34.t = adhere5->getGlobalPosition();
		adhere5->setGlobalPose(m34);
		m34.t = adhere6->getGlobalPosition();
		adhere6->setGlobalPose(m34);
		m34.t = adhere7->getGlobalPosition();
		adhere7->setGlobalPose(m34);
		m34.t = adhere8->getGlobalPosition();
		adhere8->setGlobalPose(m34);
		m34.t = adhere9->getGlobalPosition();
		adhere9->setGlobalPose(m34);

		cloth->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);
		cloth->putToSleep();
		gCloths.push_back(objCloth);
	}
}

void World::CreateHoneyComb()
{
	NxReal l = 3.0f;

	NxActor* box1 = CreateBox(NxVec3(0,0,0), NxVec3(1.5,3,0.05), NxPi/6, 5);
	NxActor* box2 = CreateBox(NxVec3(1.5 * RAD3,0,0), NxVec3(1.5,3,0.05), -NxPi/6, 5);
	NxActor* box3 = CreateBox(NxVec3(-l*RAD3/4,0, 3*l/4), NxVec3(1.5,3,0.05), NxPi/2, 5);
	NxActor* box4 = CreateBox(NxVec3(0,0,3*l/2), NxVec3(1.5,3,0.05), -NxPi/6, 5);
	NxActor* box5 = CreateBox(NxVec3(1.5 * RAD3,0,3*l/2), NxVec3(1.5,3,0.05), NxPi/6, 5);
	NxActor* box6 = CreateBox(NxVec3(3*l*RAD3/4,0,3*l/4), NxVec3(1.5,3,0.05), NxPi/2, 5);
	box1->raiseBodyFlag(NX_BF_FROZEN_POS_Y);
	box2->raiseBodyFlag(NX_BF_FROZEN_POS_Y);
	box3->raiseBodyFlag(NX_BF_FROZEN_POS_Y);
	box4->raiseBodyFlag(NX_BF_FROZEN_POS_Y);
	box5->raiseBodyFlag(NX_BF_FROZEN_POS_Y);
	box6->raiseBodyFlag(NX_BF_FROZEN_POS_Y);

	gSelectedActor = box1;

	CreateRevoluteJoint(box1, box2, NxVec3(l*RAD3/4, 0, -l/4), NxVec3(0,1,0), false);
	CreateRevoluteJoint(box1, box3, NxVec3(-l*RAD3/4,0, l/4), NxVec3(0,1,0), false);
	CreateRevoluteJoint(box3, box4, NxVec3(-l*RAD3/4,0, l*5/4), NxVec3(0,1,0), false);
	CreateRevoluteJoint(box4, box5, NxVec3(l*RAD3/4,0, 7*l/4), NxVec3(0,1,0), false);
	CreateRevoluteJoint(box5, box6, NxVec3(3*l*RAD3/4, 0, l*5/4), NxVec3(0,1,0), false);
	CreateRevoluteJoint(box6, box2, NxVec3(3*l*RAD3/4, 0, l/4), NxVec3(0,1,0), false);

	NxClothDesc clothDesc;
	clothDesc.globalPose.t = NxVec3(0.5, 0.1, 7);

	clothDesc.thickness = 0.01f;
	clothDesc.pressure = 1.0f;
	clothDesc.stretchingStiffness = 1.0f;
	clothDesc.bendingStiffness = 1.0f;
	clothDesc.friction = 0.1;
	clothDesc.solverIterations = 1;
	clothDesc.collisionResponseCoefficient = 0.2f;
	clothDesc.globalPose.M.rotY(-NxPi/6);

	clothDesc.density = 0.05f;
	clothDesc.flags |= NX_CLF_PRESSURE | NX_CLF_DAMPING;
	clothDesc.flags |= NX_CLF_BENDING | NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION | NX_CLF_TRIANGLE_COLLISION;

	MyCloth* objCloth = new MyCloth(gScene, clothDesc, "data/newmesh.obj", 0.32);
	NxCloth* cloth = objCloth->getNxCloth();

	//cloth->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);
	gSelectedCloth = cloth;
	gCloths.push_back(objCloth);
}

void World::SetupPressureScene()
{
	// Create pressure cloth
	NxClothDesc clothDesc;
	clothDesc.globalPose.t = NxVec3(-2.5,0.1,0);
	
	clothDesc.thickness = 0.01f;
	clothDesc.pressure = 1.0f;
	clothDesc.stretchingStiffness = 1.0f;
	clothDesc.bendingStiffness = 0.5f;
	clothDesc.friction = 0.1;
	clothDesc.globalPose.M.rotY(-NxPi/6);

	clothDesc.density = 0.05f;
	clothDesc.flags |= NX_CLF_PRESSURE | NX_CLF_DAMPING;
	clothDesc.flags |= NX_CLF_BENDING | NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION;

	MyCloth* objCloth = new MyCloth(gScene, clothDesc, "data/newmesh.obj", 0.3);
	NxCloth* cloth = objCloth->getNxCloth();

	
	NxClothDesc clothDesc2;
	clothDesc2.globalPose.t = NxVec3(-6.8,0.1,0);
	clothDesc2.thickness = 0.01f;
	clothDesc2.pressure = 1.0f;
	clothDesc2.stretchingStiffness = 1.0f;
	clothDesc2.bendingStiffness = 0.5f;
	clothDesc2.friction = 0.1;
	clothDesc2.globalPose.M.rotY(-NxPi/6);

	clothDesc2.density = 0.05f;
	clothDesc2.flags |= NX_CLF_PRESSURE | NX_CLF_DAMPING;
	clothDesc2.flags |= NX_CLF_BENDING | NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION;

	MyCloth* objCloth2 = new MyCloth(gScene, clothDesc2, "data/newmesh.obj", 0.3);
	NxCloth* cloth2 = objCloth2->getNxCloth();


	// cloth3
	NxClothDesc clothDesc3;
	clothDesc3.globalPose.t = NxVec3(-4.65,0.1,3.6);
	clothDesc3.thickness = 0.01f;
	clothDesc3.pressure = 1.0f;
	clothDesc3.stretchingStiffness = 1.0f;
	clothDesc3.bendingStiffness = 0.5f;
	clothDesc3.friction = 0.1;
	clothDesc3.globalPose.M.rotY(-NxPi/6);

	clothDesc3.density = 0.05f;
	clothDesc3.flags |= NX_CLF_PRESSURE | NX_CLF_DAMPING;
	clothDesc3.flags |= NX_CLF_BENDING | NX_CLF_COLLISION_TWOWAY | NX_CLF_VISUALIZATION;

	MyCloth* objCloth3 = new MyCloth(gScene, clothDesc3, "data/newmesh.obj", 0.3);
	NxCloth* cloth3 = objCloth3->getNxCloth();
	
//-------------------------test---------------------
	NxVec3 maxPos, minPos;
	NxVec3 maxPos2, minPos2;
	NxVec3 maxPos3, minPos3;
	GetPos(maxPos, minPos, cloth);
	GetPos(maxPos2, minPos2, cloth2);
	GetPos(maxPos3, minPos3, cloth3);
	printf("cloth\nmaxPos %f %f %f\nminPos %f %f %f\n\n",maxPos.x, maxPos.y, maxPos.z, minPos.x, minPos.y, minPos.z);
	printf("cloth2\nmaxPos %f %f %f\nminPos %f %f %f\n",maxPos2.x, maxPos2.y, maxPos2.z, minPos2.x, minPos2.y, minPos2.z);
	printf("cloth3\nmaxPos %f %f %f\nminPos %f %f %f\n",maxPos3.x, maxPos3.y, maxPos3.z, minPos3.x, minPos3.y, minPos3.z);
	printf("width is %f  %f, height is %f\n",maxPos.x-minPos.x, maxPos.z-minPos.z, maxPos.y-minPos.y);

	NxVec3 pos1(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2);
	NxVec3 pos2(minPos.x, minPos.y, (maxPos.z + minPos.z)/2);
	NxVec3 pos3(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2);

	NxVec3 pos4(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4);
	NxVec3 pos5(minPos.x, minPos.y, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4);
	NxVec3 pos6(minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 - (maxPos.z-minPos.z)/4);

	NxVec3 pos7(minPos.x, maxPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4); 
	NxVec3 pos8(minPos.x, minPos.y, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);
	NxVec3 pos9 (minPos.x, (maxPos.y+minPos.y)/2, (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);
	printf("pos9.z is %f\n", (maxPos.z + minPos.z)/2 + (maxPos.z-minPos.z)/4);

	NxReal adhereX1 = (minPos.x - maxPos2.x)/2;
	NxReal adhereX2 = (minPos.x - maxPos2.x)/2;

	NxActor* adhere1 = MyCreateCapsule(pos1, adhereX1, adhereX1, 1);
	NxActor* adhere2 = MyCreateCapsule(pos2, adhereX1, adhereX1, 1);
	NxActor* adhere3 = MyCreateCapsule(pos3, adhereX2, adhereX1, 1);

	NxActor* adhere4 = MyCreateCapsule(pos4, adhereX2, adhereX1, 1);
	NxActor* adhere5 = MyCreateCapsule(pos5, adhereX2, adhereX1, 1);
	NxActor* adhere6 = MyCreateCapsule(pos6, adhereX2, adhereX1, 1);

	NxActor* adhere7 = MyCreateCapsule(pos7, adhereX2, adhereX1, 1);
	NxActor* adhere8 = MyCreateCapsule(pos8, adhereX2, adhereX1, 1);
	NxActor* adhere9 = MyCreateCapsule(pos9, adhereX2, adhereX1, 1);


	NxQuat quat(90.0f, NxVec3(0,0,1));
	NxMat33 m;
	m.id();
	m.fromQuat(quat);
	NxMat34 m34;
	m34.M = m;
	m34.t = adhere1->getGlobalPosition();
	adhere1->setGlobalPose(m34);
	m34.t = adhere2->getGlobalPosition();
	adhere2->setGlobalPose(m34);
	m34.t = adhere3->getGlobalPosition();
	adhere3->setGlobalPose(m34);
	m34.t = adhere4->getGlobalPosition();
	adhere4->setGlobalPose(m34);
	m34.t = adhere5->getGlobalPosition();
	adhere5->setGlobalPose(m34);
	m34.t = adhere6->getGlobalPosition();
	adhere6->setGlobalPose(m34);
	m34.t = adhere7->getGlobalPosition();
	adhere7->setGlobalPose(m34);
	m34.t = adhere8->getGlobalPosition();
	adhere8->setGlobalPose(m34);
	m34.t = adhere9->getGlobalPosition();
	adhere9->setGlobalPose(m34);
//--------------------------------------------------
	gSelectedCloth = cloth3;

	/*cloth->putToSleep();
	cloth2->putToSleep();
	cloth3->putToSleep();
	adhere1->putToSleep();
	adhere2->putToSleep();
	adhere3->putToSleep();
	adhere4->putToSleep();
	adhere5->putToSleep();
	adhere6->putToSleep();
	adhere7->putToSleep();
	adhere8->putToSleep();
	adhere9->putToSleep();*/
	
	cloth->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);
	cloth2->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);
	cloth3->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);

	gCloths.push_back(objCloth);
	gCloths.push_back(objCloth2);
	gCloths.push_back(objCloth3);
}

void World::SetupMetalScene()
{
	// Create objects in scene
	NxActor *box1 = CreateBox(NxVec3(0,0,10), NxVec3(20,10,0.5), 0,0);
	NxActor *box2 = CreateBox(NxVec3(-20,0,0), NxVec3(0.5,10,10),0, 0);
	NxActor *box3 = CreateBox(NxVec3( 20,0,0), NxVec3(0.5,10,10), 0,0); 

	NxClothDesc clothDesc;


	// Create metal cloth with different core actors
	CreateMetalCloth(NxVec3(0.0f,  3.0f, 0.5f), 0, clothDesc, "data/barrel.obj");
	CreateMetalCloth(NxVec3(0.5f, 20.0f, 0.0f), 1, clothDesc, "data/barrel.obj");
	CreateMetalCloth(NxVec3(0.0f, 40.0f, 0.5f), 2, clothDesc, "data/barrel.obj");
	CreateMetalCloth(NxVec3(0.5f, 60.0f, 0.0f), 3, clothDesc, "data/barrel.obj");
}

void World::CreateMetalCloth(const NxVec3 &position, int mode, NxClothDesc &clothDesc, char *meshName)
{
	// Global metal data
	NxReal impulseThreshold  = 50.0f;
	NxReal penetrationDepth  = 0.5f;
	NxReal maxDeformationDistance = 0.5f;

	NxBodyDesc  coreBodyDesc;
	coreBodyDesc.linearDamping = 0.2f;
	coreBodyDesc.angularDamping = 0.2f;

	NxActorDesc coreActorDesc;
	coreActorDesc.density = 0.1f;
	coreActorDesc.body = &coreBodyDesc;

	// Create the shape, no need for size info because it is automatically generated by the cloth
	if (mode == 0) { // Sphere as core
		coreActorDesc.shapes.pushBack(new NxSphereShapeDesc());
	}
	else if (mode == 1) { // Capsule as core
		coreActorDesc.shapes.pushBack(new NxCapsuleShapeDesc());
	}
	else if (mode == 2) { // Box as core
		coreActorDesc.shapes.pushBack(new NxBoxShapeDesc());
	}
	else if (mode == 3) { // Compound of spheres as core
		const NxU32 numSpheres = 10;
		for (NxU32 i = 0; i < numSpheres; i++)
			coreActorDesc.shapes.pushBack(new NxSphereShapeDesc());
	}
	else return;

	NxActor *coreActor = gScene->createActor(coreActorDesc);
	gMetalCores.push_back(coreActor);

	// Clean up allocations
	for (NxU32 i = 0; i < coreActorDesc.shapes.size(); i++)
		delete coreActorDesc.shapes[i];

	clothDesc.globalPose.t = position;

	MyCloth *objCloth = new MyCloth(gScene, clothDesc, meshName, 1.0f);
	if (!objCloth->getNxCloth())
	{
		printf("Error: Unable to create the cloth for the current scene.\n");
		delete objCloth;
	} else
	{
		gCloths.push_back(objCloth);
		objCloth->getNxCloth()->attachToCore(coreActor, impulseThreshold, penetrationDepth, maxDeformationDistance); 
	}
}