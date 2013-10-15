#include <GL/glut.h>

#include <stdio.h>

#include "NxPhysics.h"
#include "DrawObjects.h"
#include "HUD.h"
#include "NxCooking.h"
#include "Stream.h"
#include "DebugRenderer.h"

#include "MySoftBody.h"
#include "MyCloth.h"
#include "ObjMesh.h"
#include "MediaPath.h"
#include "myforcefield.h"

#define MAX_KEYS 256
#define gNumJointTypesOfFreedom 3U
#define gNumJointDegreesOfFreedom 6U
#define gNumJointConfigurations 3U

static bool gKeys[MAX_KEYS];


class World
{
public:
	
	World();

	static void MyPrintControls();
	static void MyProcessCameraKeys();
	static void MySetupCamera();
	static void MyRenderActors(bool shadows);
	static void MyDrawForce(NxActor* actor, NxVec3& forceVec, const NxVec3& color);

	static NxVec3 MyApplyForceToActor(NxActor* actor, const NxVec3& forceDir, const NxReal forceStrength, bool forceMode);
	static void MyProcessForceKeys();
	static void MyProcessInputs();
	static void MySelectNextActor();
	static void MySelectNextCloth();
	static bool MyIsSelectable(NxActor* actor);

	static void MyRenderCallback();
	static void MyReshapeCallback(int width, int height);
	static void MyIdleCallback();
	static void MyKeyboardCallback(unsigned char key, int x, int y);
	static void MyKeyboardUpCallback(unsigned char key, int x, int y);
	static void MySpecialCallback(int key, int x, int y);
	static void MyMouseCallback(int button, int state, int x, int y);
	static void MyMotionCallback(int x, int y);
	static void MyExitCallback();
	void MyInitGlut(int argc, char** argv);

	static NxActor* MyCreateGroundPlane();
	static NxActor* MyCreateBox();
	static NxActor* MyCreateCapsule(const NxVec3& pos, NxReal height, NxReal radius, NxReal density);
	static NxActor* MyCreateConvexMesh();
	static NxActor* MyCreateTriangleMesh();

	static NxActor* MyCreatePrimaryMultiShape();
	static NxActor* MyCreateMeshMultiShape();
	static NxActor* MyCreateDynamicActor();
	static NxActor* MyCreateKinematicActor();

	static NxActor* MyCreateBounds3();
	static NxActor* MyCreateSegment();
	static NxActor* MyCreateRay();

	static void MyCreateStack();
	static NxActor* CreateBox(const NxVec3& pos, const NxVec3& boxDim, const NxReal density);
	static NxActor* CreateSphere(const NxVec3& pos, const NxReal radius, const NxReal density);
	static NxActor* CreateDownWedge(const NxVec3& pos, const NxVec3& boxDim, const NxReal density);
	static NxActor* CreateRegularPyramid(const NxVec3& pos, const NxVec3& boxDim, const NxReal density);
	static NxActor* CreateCompoundCapsule(const NxVec3& pos0, const NxReal height0, const NxReal radius0, const NxReal height1, const NxReal radius1);
	static NxActor* CreateOrientedCapsule(const NxMat34& globalPose, const NxReal height, const NxReal radius, const NxReal density);

	static NxFluid* CreateFluid(const NxVec3& pos, NxU32 sideNum, NxReal distance, NxScene* scene);
	static void ReleaseFluid();
	static void RenderFluid();
	static void DrawFluid(NxFluid* fluid);

	static void SetupAnimalScene();
	static void RenderSoftBody();
	static void RenderCloth();

	// Create joint
	static NxRevoluteJoint* CreateRevoluteJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis, bool isSpring);
	static NxFixedJoint* CreateFixedJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	static NxSphericalJoint* CreateSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis);
	static NxPrismaticJoint* CreatePrismaticJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	static NxCylindricalJoint* CreateCylindricalJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	static NxPointOnLineJoint* CreatePointOnLineJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	static NxPointInPlaneJoint* CreatePointInPlaneJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	static NxPulleyJoint* CreatePulleyJoint(NxActor* a0, NxActor* a1, const NxVec3& pulley0, const NxVec3& pulley1, 
											const NxVec3& globalAxis, NxReal distance, NxReal ratio, const NxMotorDesc& motorDesc);
	static NxDistanceJoint* CreateDistanceJoint(NxActor* a0, NxActor* a1, const NxVec3& anchor0, const NxVec3& anchor1, const NxVec3& globalAxis);
	static NxD6Joint* CreateD6Joint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis);
	static void ReconfigureD6Joint(NxD6Joint* d6Joint, NxActor* a0, NxActor* a1);
	static NxSphericalJoint* CreateRopeSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis);
	

	// Create softbody
	static void SetupSoftBodyCube();
	static void SetupPalmScene();
	static void SetupBunnyScene();
	static void SetupSoftWheelCarScene();

	// Create cloth
	static void SetupCurtainScene();
	static void SetupFlagScene();
	static void SetupPressureScene();
	static void CreateSoftComb();
	static void SetupMetalScene();
	static void CreateMetalCloth(const NxVec3 &position, int mode, NxClothDesc &clothDesc, char *meshName);



	static void MyInitializeHUD();

	static void MyInitNx();
	static void MyReleaseNx();
	static void MyResetNx();

	static void MyStartPhysics();
	static void MyGetPhysicsResults();



	// Physics SDK globals
	static NxPhysicsSDK*     gPhysicsSDK;
	static NxScene*          gScene;
	static NxVec3            gDefaultGravity;

	// User report globals
	static DebugRenderer     gDebugRenderer;

	// HUD globals
	static HUD hud;
 
	// Display globals
	static int gMainHandle;
	static int mx;
	static int my;

	// Camera globals
	static float	gCameraAspectRatio;
	static NxVec3	gCameraPos;
	static NxVec3	gCameraForward;
	static NxVec3	gCameraRight;
	static NxReal	gCameraSpeed;

	// Force globals
	static NxVec3	gForceVec;
	static NxReal	gForceStrength;
	static bool	bForceMode;

	// Simulation globals
	static NxReal	gDeltaTime;
	static bool	bHardwareScene;
	static bool	bPause;
	static bool	bShadows;
	static bool	bDebugWireframeMode;

	// Actor globals
	static NxActor* groundPlane;
	static NxConvexMeshDesc* convexDesc;
	static NxTriangleMeshDesc* triangleMeshDesc;
	static NxTriangleMeshDesc* concaveDesc;

	// Focus actor
	static NxActor* gSelectedActor;

	static NxArray<MySoftBody *>	gSoftBodies;
	static NxArray<MyCloth *>		gCloths;
	static NxArray<ObjMesh *>		gObjMeshes;
	static NxArray<NxActor*, NxAllocatorDefault> gMetalCores;
	static int gInvisible;

	// ForceField
	static NxArray<MyForceField*>	gSamples;
	static unsigned gSampleIndex;

	// Joint globals
	static NxRevoluteJoint* revJoint;
	static NxFixedJoint* fixedJoint;
	static bool bReconfigureD6Joint;
	static NxU32 gJointType;
	static NxD6JointMotion gJointMotion[gNumJointDegreesOfFreedom];
	static char* gJointTypeString[gNumJointConfigurations];
	static char* gJointMotionString[gNumJointTypesOfFreedom];

	// SoftBody globals
	static NxSoftBody* gSelectedSoftBody;
	static NxCloth* gSelectedCloth;



	static World* world;

};


// test for initnx cases
void testSphericalJoint(World* world);
void testPrismaticJoints(World* world);
void testCylindricalJoint(World* world);
void testPointOnLineJoint(World* world);
void testPointInPlaneJoint(World* world);
void testPulleyJoint(World* world);
void testDistanceJoint(World* world);