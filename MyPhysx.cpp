#include "stdafx.h"
#include <iostream>
#include "MyPhysx.h"
#include "UpdateTime.h"

using namespace std;

#define REST_PARTICLES_PER_METER 10
#define KERNEL_RADIUS_MULTIPLIER 1.8
#define MOTION_LIMIT_MULTIPLIER 3
#define PACKET_SIZE_MULTIPLIER 8


World* World::world;

// Physics SDK globals
NxPhysicsSDK*     World::gPhysicsSDK;
NxScene*          World::gScene;
NxVec3            World::gDefaultGravity;

// User report globals
DebugRenderer     World::gDebugRenderer;

// HUD globals
HUD World::hud;
 
// Display globals
int World::gMainHandle;
int World::mx;
int World::my;

// Camera globals
float	World::gCameraAspectRatio;
NxVec3	World::gCameraPos;
NxVec3	World::gCameraForward;
NxVec3	World::gCameraRight;
NxReal	World::gCameraSpeed;

// Force globals
NxVec3	World::gForceVec;
NxReal	World::gForceStrength;
bool	World::bForceMode;

// Simulation globals
NxReal	World::gDeltaTime;
bool	World::bHardwareScene;
bool	World::bPause;
bool	World::bShadows;
bool	World::bDebugWireframeMode;

// Actor globals
NxActor* World::groundPlane;
NxConvexMeshDesc* World::convexDesc;
NxTriangleMeshDesc* World::triangleMeshDesc;
NxTriangleMeshDesc* World::concaveDesc;

// Fluid globals
NxFluid* fluid = NULL;

// Fluid particle globals
NxVec3 gParticleBuffer[10000];
NxU32 gParticleBufferCap = 10000;
NxU32 gParticleBufferNum = 0;


// ForceField stuff
NxArray<MyForceField*> World::gSamples;
unsigned World::gSampleIndex = 0;

// Joint globals
NxRevoluteJoint* World::revJoint;
NxFixedJoint* World::fixedJoint = NULL; 
bool World::bReconfigureD6Joint;
NxU32 World::gJointType;
char* World::gJointTypeString[gNumJointConfigurations];
NxD6JointMotion World::gJointMotion[gNumJointDegreesOfFreedom];
char* World::gJointMotionString[gNumJointTypesOfFreedom];

// Mesh globals
NxTriangleMeshDesc flatSurfaceTriangleMeshDesc;
NxArray<MySoftBody *>	World::gSoftBodies;
NxArray<MyCloth *>		World::gCloths;
NxArray<ObjMesh *>		World::gObjMeshes;
int World::gInvisible;

// Focus actor
NxActor* World::gSelectedActor;
NxSoftBody* World::gSelectedSoftBody;
NxCloth* World::gSelectedCloth;


World::World()
{
	world = this;

	// Physics SDK globals
	gPhysicsSDK = NULL;
	gScene = NULL;
	gDefaultGravity = NxVec3(0,-9.8,0);

	// Display globals
	gMainHandle = 0;
	mx = 0;
	my = 0;

	gSampleIndex = 0;

	// Camera globals
	gCameraAspectRatio = 1.0f;
	gCameraPos = NxVec3(-2,20,-28);
	gCameraForward = NxVec3(0,0,1);
	gCameraRight = NxVec3(-1,0,0);
	gCameraSpeed = 20;

	// Force globals
	gForceVec = NxVec3(0,0,0);
	gForceStrength	= 20000;
	bForceMode		= true;


	// Simulation globals
	gDeltaTime			= 1.0/60.0;
	bHardwareScene		= true;
	bPause				= false;
	bShadows			= true;
	bDebugWireframeMode = false;

	// Actor globals
	groundPlane = NULL;
	convexDesc	= NULL;
	triangleMeshDesc = NULL;
	concaveDesc		 = NULL;

	revJoint = NULL;

	// Focus actor
	gSelectedActor = NULL;
	gSelectedSoftBody = NULL;

	bReconfigureD6Joint = false;
	gJointType = 0;

	gJointTypeString[0] = "Fixed";
	gJointTypeString[1] = "Revolute";
	gJointTypeString[2] = "Spherical";

	gJointMotionString[0] = "Locked";
	gJointMotionString[0] = "Limited";
	gJointMotionString[0] = "Free";


	for (int i = 0; i < gNumJointDegreesOfFreedom; i++)
		gJointMotion[i] = NX_D6JOINT_MOTION_LOCKED;
}


void World::MyPrintControls()
{
	cout << "Hello World!\n" <<endl;
	cout << "PhysX Test "<<endl;
}

void World::MyInitGlut(int argc, char** argv)
{
	glutInit(&argc, argv);
    glutInitWindowSize(768, 768);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    gMainHandle = glutCreateWindow("My PhysX");
    glutSetWindow(gMainHandle);
    glutDisplayFunc(MyRenderCallback);	//需要显示时调用
    glutReshapeFunc(MyReshapeCallback);	//窗口大小变化时调用
    glutIdleFunc(MyIdleCallback);			//没窗体事件产生时调用
    glutKeyboardFunc(MyKeyboardCallback);	//键盘事件
    glutKeyboardUpFunc(MyKeyboardUpCallback);	//按键弹起
	glutSpecialFunc(MySpecialCallback);
    glutMouseFunc(MyMouseCallback);
    glutMotionFunc(MyMotionCallback);
	MyMotionCallback(0,0);
	atexit(MyExitCallback);

    // Setup default render states
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);

    // Setup lighting
    glEnable(GL_LIGHTING);
    float AmbientColor[]    = { 0.0f, 0.1f, 0.2f, 0.0f };         glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
    float DiffuseColor[]    = { 0.2f, 0.2f, 0.2f, 0.0f };         glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
    float SpecularColor[]   = { 0.5f, 0.5f, 0.5f, 0.0f };         glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);
    float Position[]        = { 100.0f, 100.0f, -400.0f, 1.0f };  glLightfv(GL_LIGHT0, GL_POSITION, Position);
    glEnable(GL_LIGHT0);
}

void InitActorCollisionGroups()
{

}

void World::MyInitNx()
{
	// Initialize camera parameters
	gCameraAspectRatio	= 1.0f;
	gCameraPos			= NxVec3(0,12,-28);
	gCameraForward		= NxVec3(0,0,1);
	gCameraRight		= NxVec3(-1,0,0);
	
	// Create the physics SDK
    gPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
    if (!gPhysicsSDK)  return;
	NxHWVersion hwCheck = gPhysicsSDK->getHWVersion();
	if (hwCheck == NX_HW_VERSION_NONE)
	{
		printf("\nWarning: Unable to find a PhysX card, or PhysX card used by other application.");
		printf("\nThe soft bodies will be simulated in software.\n\n");
		//gHardwareSoftBodySimulation = false;
		bHardwareScene = false;
	}

	NxInitCooking();

	// Set the physics parameters
	gPhysicsSDK->setParameter(NX_SKIN_WIDTH, 0.01);
	gPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_LIN_VEL_SQUARED, 0.15*0.15);
	gPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_ANG_VEL_SQUARED, 0.14*0.14);

	// Set the debug visualization parameters
	gPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES, 1);

    // Create the scene
    NxSceneDesc sceneDesc;
 	sceneDesc.simType				= NX_SIMULATION_SW;
    sceneDesc.gravity               = gDefaultGravity;
    gScene = gPhysicsSDK->createScene(sceneDesc);	
	if(!gScene)
	{ 
		sceneDesc.simType			= NX_SIMULATION_SW;
		gScene = gPhysicsSDK->createScene(sceneDesc);
		if(!gScene) 
		{
			printf("gScene cannot be created.\n");
			exit(0);
		}
	}
	

	// Create the default material
	NxMaterial* defaultMaterial = gScene->getMaterialFromIndex(0); 
	defaultMaterial->setRestitution(0.5);
	defaultMaterial->setStaticFriction(0.5);
	defaultMaterial->setDynamicFriction(0.5);
	 
	// Create the objects in the scene
	groundPlane		= MyCreateGroundPlane();
	//NxActor* box1	= MyCreateBox();
	//gSelectedActor = box1;
	/*box1->raiseBodyFlag(NX_BF_FROZEN_POS_X);
	box1->raiseBodyFlag(NX_BF_FROZEN_POS_Z);
	box1->setLinearDamping(0.5);*/

	//NxActor* box1 = CreateBox(NxVec3(0,5,0), NxVec3(0.5,2,1),10);
	//box1->raiseBodyFlag(NX_BF_KINEMATIC);
	//NxActor* box2 = CreateBox(NxVec3(0,1,0), NxVec3(0.5,2,1), 10);
	//box2->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	//
	NxVec3 globalAnchor = NxVec3(0.5, 5, -1);
	NxVec3 globalAxis = NxVec3(0,1,0);

	//fixedJoint = CreateFixedJoint(box1, box2, globalAnchor, globalAxis);
	//revJoint = CreateRevoluteJoint(box1, box2, globalAnchor, globalAxis, false);

	
	//testSphericalJoint(World::world);
	//testPrismaticJoints(World::world);
	//testCylindricalJoint(World::world);
	//testPointOnLineJoint(World::world);
	//testPointInPlaneJoint(World::world);
	//testPulleyJoint(World::world);
	//testDistanceJoint(World::world);
	//SetupSoftBodyCube();
	//SetupPalmScene();
	//SetupBunnyScene();
	//SetupSoftWheelCarScene();
	//SetupCurtainScene();
	//SetupFlagScene();
	SetupPressureScene();
	//SetupMetalScene();

	//fluid = CreateFluid(NxVec3(0,3,0), 15, 0.1, gScene);

	// Create the objects in the scene
	//SetupAnimalScene();

	// Create ForceField

	//gSampleIndex = 2;
	//gSamples[gSampleIndex]->setup(gScene);

	
	//MyCreateMeshMultiShape();
	//MyCreateConvexMesh();
	//MyCreateTriangleMesh();
	//MyCreatePrimaryMultiShape();
	//MyCreateMeshMultiShape();
	//gSelectedActor = CreateSphere();
	//gSelectedActor->raiseBodyFlag(NX_BF_ENERGY_SLEEP_TEST);
	//gSelectedActor->raiseBodyFlag(NX_BF_FILTER_SLEEP_VEL);
	
	// Create box stack
	//MyCreateStack();

	//bPause = true;

	// Initialize HUD
	MyInitializeHUD();

	// Get the current time
	UpdateTime();

	// Start the first frame of the simulation
	if (gScene)  MyStartPhysics();
}

static bool flag = true;

void World::MyRenderCallback()
{
    // Clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    MyProcessCameraKeys();
	MySetupCamera();


    if (gScene && !bPause)
	{
		MyGetPhysicsResults();
        MyProcessInputs();
		MyStartPhysics();

		/*NxReal curPressure = gCloths[0]->getNxCloth()->getPressure();

		printf("%f              %d\n", curPressure, flag);
		if (curPressure > 0.01 && flag == true)
		{
			gCloths[0]->getNxCloth()->setPressure(curPressure-0.01);
			printf("curPressure is %f\n", curPressure);
			if (curPressure-0.01 < 0.01)
				flag = false;
		}
		else if (curPressure < 4.5 && flag == false)
		{
			gCloths[0]->getNxCloth()->setPressure(curPressure+0.01);
			printf("curPressure is %f\n", curPressure);
			if (curPressure+0.01 > 4.5)
				flag = true;
		}*/
		
	}

    // Display scene
 	MyRenderActors(bShadows);

	RenderSoftBody();
	RenderCloth();

	//RenderFluid();

	//DrawForce(gSelectedActor, gForceVec, NxVec3(1,1,0));
	gForceVec = NxVec3(0,0,0);

	// Render the HUD
	hud.Render();

    glFlush();
    glutSwapBuffers();
}

void World::MyReshapeCallback(int width, int height)
{
    glViewport(0, 0, width, height);
    gCameraAspectRatio = float(width)/float(height);
}

void World::MyIdleCallback()
{
    glutPostRedisplay();
}

void World::MyKeyboardCallback(unsigned char key, int x, int y)
{
	gKeys[key] = true;

	switch (key)
	{
		case 'r':	{ MySelectNextActor(); break; }
		case 't':	{ MySelectNextCloth(); break; }
		case 'e':
			{
				if(gSelectedActor)
					gSelectedActor->setGlobalPosition(NxVec3(3.0f, 5.0f, 3.0f));
				break;
			}

		case 'c':
			{
				if(gSelectedActor)
					gSelectedActor->setLinearVelocity(NxVec3(1.0f, 0, 100));
				break;
			}
		case 'v':
			{
				if(gSelectedActor)
					gSelectedActor->setAngularVelocity(NxVec3(0, 1000.0f, 0));
				break;
			}
		case '+':
			{
				if(gSelectedCloth)
				{
					printf("current pressure is %f\n", gSelectedCloth->getPressure());
					gSelectedCloth->setPressure(gSelectedCloth->getPressure()+0.1);
					printf("after increasing pressure is %f\n", gSelectedCloth->getPressure());

				}
				break;
			}
		case '-':
			{
				if(gSelectedCloth)
				{
					printf("current pressure is %f\n", gSelectedCloth->getPressure());
					gSelectedCloth->setPressure(gSelectedCloth->getPressure()-0.1);
					printf("after decreasing pressure is %f\n", gSelectedCloth->getPressure());
				}
				break;
			}
		default:	{ break; }
	}
}

void World::MyKeyboardUpCallback(unsigned char key, int x, int y)
{
	gKeys[key] = false;

	switch (key)
	{
		case 'p': 
		{ 
			bPause = !bPause; 
			if (bPause)
				hud.SetDisplayString(0, "Paused - Hit \"p\" to Unpause", 0.3f, 0.55f);
			else
				hud.SetDisplayString(0, "", 0.0f, 0.0f);	
			UpdateTime(); 
			break; 
		}
		case 'x': { bShadows = !bShadows; break; }
		case 'b': { bDebugWireframeMode = !bDebugWireframeMode; break; }
		case 'f': {	bForceMode = !bForceMode; break;}
		case 27 : { exit(0); break; }
		default : { break; }
	}
}

void World::MySpecialCallback(int key, int x, int y)
{
	switch (key)
    {
		// Reset PhysX
		case GLUT_KEY_F10: MyResetNx(); return; 
	}
}

void World::MyMouseCallback(int button, int state, int x, int y)
{
    mx = x;
    my = y;
}

void World::MyMotionCallback(int x, int y)
{
    int dx = mx - x;
    int dy = my - y;
    
    gCameraForward.normalize();
    gCameraRight.cross(gCameraForward,NxVec3(0,1,0));

    NxQuat qx(NxPiF32 * dx * 20 / 180.0f, NxVec3(0,1,0));
    qx.rotate(gCameraForward);
    NxQuat qy(NxPiF32 * dy * 20 / 180.0f, gCameraRight);
    qy.rotate(gCameraForward);

    mx = x;
    my = y;
}

void World::MyExitCallback()
{
	MyReleaseNx();
}

void World::MyStartPhysics()
{
	// Update the time step
	gDeltaTime = UpdateTime();

	// Start collision and dynamics for delta time since the last frame
    gScene->simulate(gDeltaTime);
	gScene->flushStream();
}

void World::MyProcessInputs()
{
	gKeys['t'] = false;		// will replace with method DisableAKey(int key)
	MyProcessForceKeys();

	if (bReconfigureD6Joint)
	{
		bReconfigureD6Joint = false;
		gJointType = (gJointType+1)%gNumJointConfigurations;
		//ReconfigureD6Joint();
	}

    // Show debug wireframes
	if (bDebugWireframeMode)
	{
		if (gScene)  gDebugRenderer.renderData(*gScene->getDebugRenderable());
	}
}

void World::MyInitializeHUD()
{
	bHardwareScene = (gScene->getSimType() == NX_SIMULATION_HW);

	hud.Clear();

	// Add hardware/software to HUD
	if (bHardwareScene)
	    hud.AddDisplayString("Hardware Scene", 0.74f, 0.92f);
	else
		hud.AddDisplayString("Software Scene", 0.74f, 0.92f);

	// Add pause to HUD
	if (bPause)  
		hud.AddDisplayString("Paused - Hit \"p\" to Unpause", 0.3f, 0.55f);
	else
		hud.AddDisplayString("", 0.0f, 0.0f);
}



NxFluid* World::CreateFluid(const NxVec3& pos, NxU32 sideNum, NxReal distance, NxScene* scene)
{
	// pos = (0 2 0), sideNum = 15, distance = 0.1, gScene
	// Create a set of particles
	gParticleBufferNum = 0;
	NxReal rad = sideNum * distance * 0.5;		// rad = 15 * 0.1 * 0.5 = 0.75
	for (NxU32 i = 0; i < sideNum; i++)
	{
		for (NxU32 j = 0; j < sideNum; j++)
		{
			for (NxU32 k = 0; k < sideNum; k++)
			{
				NxVec3 p = NxVec3(i*distance, j*distance, k*distance);
				if (p.distance(NxVec3(rad, rad, rad)) < rad)
				{
					p += pos - NxVec3(rad, rad, rad);
					gParticleBuffer[gParticleBufferNum++] = p;
				}
			}
		}
	}

	// Set structure to pass particles, and receive them after every simulation step
	NxParticleData particles;
	particles.numParticlesPtr		= &gParticleBufferNum;
	particles.bufferPos				= &gParticleBuffer[0].x;
	particles.bufferPosByteStride	= sizeof(NxVec3);

	// Create a fluid descriptor
	NxFluidDesc fluidDesc;
	fluidDesc.maxParticles						= gParticleBufferCap;
	fluidDesc.kernelRadiusMultiplier			= KERNEL_RADIUS_MULTIPLIER;
	fluidDesc.restParticlesPerMeter				= REST_PARTICLES_PER_METER;
	fluidDesc.motionLimitMultiplier				= MOTION_LIMIT_MULTIPLIER;
	fluidDesc.packetSizeMultiplier				= PACKET_SIZE_MULTIPLIER;
	fluidDesc.stiffness							= 50;
	fluidDesc.viscosity							= 22;
	fluidDesc.restDensity						= 1000;
	fluidDesc.damping							= 0;
	fluidDesc.restitutionForDynamicShapes		= 0.4;
	fluidDesc.dynamicFrictionForDynamicShapes	= 0.3;
	fluidDesc.collisionResponseCoefficient		= 0.5f;
	fluidDesc.collisionDistanceMultiplier		= 0.1f;
	fluidDesc.simulationMethod					= NX_F_SPH;

	fluidDesc.initialParticleData				= particles;
	fluidDesc.particlesWriteData				= particles;
	fluidDesc.flags								&= ~NX_FF_HARDWARE;
	fluidDesc.flags								|= NX_FF_COLLISION_TWOWAY;

	NxFluid* fl = gScene->createFluid(fluidDesc);
	assert(fl);

	return NULL;
}

void World::SetupAnimalScene()
{
	// Create the objects in the scene
	NxSoftBodyDesc softBodyDesc;
	softBodyDesc.globalPose.t		= NxVec3(0.0f, 3.0f, 0.0f);
	softBodyDesc.particleRadius		= 0.2f;
	softBodyDesc.volumeStiffness	= 0.5f;
	softBodyDesc.stretchingStiffness= 1.0f;
	softBodyDesc.friction			= 1.0f;
	softBodyDesc.attachmentResponseCoefficient = 0.1f;
	softBodyDesc.solverIterations	= 5;

	char *fileName = "froggyNormalized";

	char tetFileName[256], objFileName[256], s[256];
	sprintf(tetFileName, "%s.tet", fileName);
	sprintf(objFileName, "%s.obj", fileName);

	ObjMesh *objMesh = new ObjMesh();
	objMesh->loadFromObjFile(FindMediaFile(objFileName, s));
	gObjMeshes.pushBack(objMesh);

	if(objMesh == NULL)
	{
		printf("objMesh %s\n");
	}

	NxMat33 rot;
	rot.rotX(NxHalfPiF32);

	for (int i = 0; i < 5; i++)
	{
		softBodyDesc.globalPose.t	= NxVec3(0, 3+i*3, 0);
		MySoftBody *softBody		= new MySoftBody(gScene, softBodyDesc, FindMediaFile(tetFileName, s), objMesh);
		assert(softBody);
		if (!softBody->getNxSoftBody())
		{
			printf("Error: unable to create the softbody for the current scene.\n");
			delete softBody;
		}
		else
		{
			gSoftBodies.pushBack(softBody);
			NxActor *caps = MyCreateCapsule(NxVec3(0.0f, 3.0f + i*3.0f, -0.3f), 1.0f, 0.73f, 1.0f);
			caps->userData = (void*)&gInvisible;
			caps->setGlobalOrientation(rot);
			softBody->getNxSoftBody()->attachToShape(caps->getShapes()[0], NX_SOFTBODY_ATTACHMENT_TWOWAY);
		}
	}
}

void World::ReleaseFluid()
{
	if (fluid)
	{
		if (gScene)	gScene->releaseFluid(*fluid);
		fluid = NULL;
	}
}

void DrawActorBlueSleep(NxActor *actor, NxActor* selectedActor)
{
	NxShape*const* shapes = actor->getShapes();
	NxU32 nShapes = actor->getNbShapes();
	if (actor == selectedActor) 
	{
		while (nShapes--)
		{
			DrawWireShape(shapes[nShapes], NxVec3(1,1,1), false);
		}
	}
	if (actor->isSleeping())
	{
		glDisable(GL_LIGHTING);
		glColor4f(0.0f,0.0f,1.0f,1.0f);
	}
	nShapes = actor->getNbShapes();
	while (nShapes--)
	{
		DrawShape(shapes[nShapes], false);
	}
	if (actor->isSleeping())
	{
		glEnable(GL_LIGHTING);
	}
}

void World::MyRenderActors(bool shadows)
{
    // Render all the actors in the scene
	if (gScene == NULL)
		printf("what ??\n");
    NxU32 nbActors = gScene->getNbActors();
    NxActor** actors = gScene->getActors();
    while (nbActors--)
    {
        NxActor* actor = *actors++;
        DrawActor(actor, gSelectedActor, true);
		//DrawActorBlueSleep(actor, gSelectedActor);

        // Handle shadows
        if (shadows)
        {
			DrawActorShadow(actor, true);
        }
    }
}

void World::RenderSoftBody()
{
	MySoftBody** softBody = gSoftBodies.begin();
	for (; softBody != gSoftBodies.end(); softBody++)
	{
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		(*softBody)->simulateAndDraw(false, true, false);
	}
}

void World::RenderCloth()
{
	for (MyCloth **cloth = gCloths.begin(); cloth != gCloths.end(); cloth++)
	{
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		(*cloth)->draw(bShadows);
	}
}

void World::RenderFluid()
{
	NxFluid** fluids = gScene->getFluids();
	int nbFluids = gScene->getNbFluids();
	while (nbFluids--)
	{
		NxFluid* fluid = *fluids++;
		DrawFluid(fluid);
	}

	// Keep physics & graphics in sync
	for (NxU32 p = 0; p < gParticleBufferNum; p++)
	{
		NxVec3& particle = gParticleBuffer[p];

		glPushMatrix();
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glTranslatef(particle.x, particle.y, particle.z);
		glutSolidCube(0.1);
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		glPopMatrix();
	}
}

void World::DrawFluid(NxFluid* fluid)
{
#ifndef NX_DISABLE_FLUIDS
	NxParticleData particles = fluid->getParticlesWriteData();
	if (!particles.numParticlesPtr)
		return;

	NxU32 numParticles = *particles.numParticlesPtr;
	if (numParticles == 0)
		return;

#ifndef __CELLOS_LV2__
	glPushAttrib(GL_ALL_ATTRIB_BITS);
#endif

	glPointSize(5.0f);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, particles.bufferPosByteStride, particles.bufferPos);

	glDrawArrays(GL_POINTS, 0, numParticles);

	glDisableClientState(GL_VERTEX_ARRAY);

#ifndef __CELLOS_LV2__
	glPopAttrib();
#endif

#endif	//NX_DISABLE_FLUIDS
}

void World::MyGetPhysicsResults()
{
	// Get results from gScene->simulate(gDeltaTime)
	while (!gScene->fetchResults(NX_RIGID_BODY_FINISHED, false));
}

void World::MySetupCamera()
{
	gCameraAspectRatio = (float)glutGet(GLUT_WINDOW_WIDTH) / (float)glutGet(GLUT_WINDOW_HEIGHT);
	
	// Setup camera
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, gCameraAspectRatio, 1.0f, 10000.0f);
	gluLookAt(gCameraPos.x,gCameraPos.y,gCameraPos.z,gCameraPos.x + gCameraForward.x, gCameraPos.y + gCameraForward.y, gCameraPos.z + gCameraForward.z, 0.0f, 1.0f, 0.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void World::MyProcessCameraKeys()
{
	NxReal deltaTime;

    if (bPause) 
	{
		deltaTime = 0.0005;
	}
	else 
	{
		deltaTime = gDeltaTime;
	}
		
	// Process camera keys
	for (int i = 0; i < MAX_KEYS; i++)
	{	
		if (!gKeys[i])  { continue; }

		switch (i)
		{
			// Camera controls
			case 'w':{ gCameraPos += gCameraForward*gCameraSpeed*deltaTime; break; }
			case 's':{ gCameraPos -= gCameraForward*gCameraSpeed*deltaTime; break; }
			case 'a':{ gCameraPos -= gCameraRight*gCameraSpeed*deltaTime;	break; }
			case 'd':{ gCameraPos += gCameraRight*gCameraSpeed*deltaTime;	break; }
			case 'z':{ gCameraPos -= NxVec3(0,1,0)*gCameraSpeed*deltaTime;	break; }
			case 'q':{ gCameraPos += NxVec3(0,1,0)*gCameraSpeed*deltaTime;	break; }
		}
	}
}

void World::MySelectNextActor()
{
   NxU32 nbActors = gScene->getNbActors();
   NxActor** actors = gScene->getActors();
   for(NxU32 i = 0; i < nbActors; i++)
   {
       if (actors[i] == gSelectedActor)
       {
           NxU32 j = 1;
           gSelectedActor = actors[(i+j)%nbActors];
           while (!MyIsSelectable(gSelectedActor))
           {
               j++;
               gSelectedActor = actors[(i+j)%nbActors];
           }
           break;
       }
   }
}

void World::MySelectNextCloth()
{
	for (NxU32 i = 0; i < gCloths.size(); i++)
	{
		if (gCloths[i]->getNxCloth() == gSelectedCloth)
		{
			gSelectedCloth = gCloths[(i+1) % gCloths.size()]->getNxCloth();
			printf("select %d,  %d total\n", (i+1) % gCloths.size(), gCloths.size());
			return;
		}
	}
}

void World::MyProcessForceKeys()
{
	// Process force keys
	for (int i = 0; i < MAX_KEYS; i++)
	{	
		if (!gKeys[i])  { continue; }

		switch (i)
		{
			// Force controls
			case 'i': { gForceVec = MyApplyForceToActor(gSelectedActor,NxVec3(0,0,1),gForceStrength, bForceMode);		break; }
			case 'k': { gForceVec = MyApplyForceToActor(gSelectedActor,NxVec3(0,0,-1),gForceStrength, bForceMode);	break; }
			case 'j': { gForceVec = MyApplyForceToActor(gSelectedActor,NxVec3(1,0,0),gForceStrength, bForceMode);		break; }
			case 'l': { gForceVec = MyApplyForceToActor(gSelectedActor,NxVec3(-1,0,0),gForceStrength, bForceMode);	break; }
			case 'u': { gForceVec = MyApplyForceToActor(gSelectedActor,NxVec3(0,1,0),gForceStrength, bForceMode);		break; }
			case 'm': { gForceVec = MyApplyForceToActor(gSelectedActor,NxVec3(0,-1,0),gForceStrength, bForceMode);	break; }

		    // Return box to (0,5,0)
			case 't': 
			{ 
				if (gSelectedActor) 
				{
					gSelectedActor->setGlobalPosition(NxVec3(0,5,0)); 
					gScene->flushCaches();
				}
				break; 
			}
		}
	}
}

bool World::MyIsSelectable(NxActor* actor)
{
   NxShape*const* shapes = gSelectedActor->getShapes();
   NxU32 nShapes = gSelectedActor->getNbShapes();
   while (nShapes--)
   {
       if (shapes[nShapes]->getFlag(NX_TRIGGER_ENABLE)) 
       {           
           return false;
       }
   }

   if(!actor->isDynamic())
	   return false;

   if (actor == groundPlane)
       return false;

   return true;
}

void World::MyReleaseNx()
{
	MyGetPhysicsResults();

	gSamples[gSampleIndex]->cleanup();

    if (gScene)
	{
		MyGetPhysicsResults();  // Make sure to fetchResults() before shutting down
		gPhysicsSDK->releaseScene(*gScene);
	}
	if (gPhysicsSDK)  gPhysicsSDK->release();
}

void World::MyResetNx()
{
	MyReleaseNx();
	MyInitNx();
}

NxVec3 World::MyApplyForceToActor(NxActor* actor, const NxVec3& forceDir, const NxReal forceStrength, bool forceMode)
{
	NxVec3 forceVec = forceStrength*forceDir*gDeltaTime;

	if(!actor)
		return NULL;

	if (forceMode)
		actor->addForce(forceVec);
	else
		actor->addTorque(forceVec);

	return forceVec;
}

int main(int argc, char** argv)
{
	World world;

	world.gSamples.pushBack(new SampleVortex());
	world.gSamples.pushBack(new SampleWind());
	world.gSamples.pushBack(new SampleExplosion());

	world.MyPrintControls();
	world.MyInitGlut(argc, argv);
	world.MyInitNx();
	glutMainLoop();
	world.MyReleaseNx();
	return 0;
}