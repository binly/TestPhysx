#include "stdafx.h"
#include "MyPhysx.h"

NxActor* World::MyCreateBox()
{
	// Set the box starting height to 3.5m so box starts off falling onto the ground
	NxReal boxStartHeight = 3.5; 

	// Add a single-shape actor to the scene
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// The actor has one shape, a box, 1m on a side
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions.set(0.5,1.0,0.5);
	boxDesc.localPose.t = NxVec3(0, 5, 0);

	actorDesc.shapes.pushBack(&boxDesc);

	actorDesc.body = &bodyDesc;
	actorDesc.density = 5.0f;
	actorDesc.globalPose.t = NxVec3(0, boxStartHeight, 0);
	assert(actorDesc.isValid());

	NxActor *pActor = gScene->createActor(actorDesc);
	assert(pActor);

	return pActor;
}

NxActor* World::MyCreateCapsule(const NxVec3& pos, NxReal height, NxReal radius, NxReal density)
{
	// Add a single-shape actor to the scene
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// The actor has one shape, a capsule
	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.height		= height;
	capsuleDesc.radius		= radius;
	capsuleDesc.localPose.t	= NxVec3(0, radius+(NxReal)0.5*height, 0);
	
	//capsuleDesc.localPose.M.rotZ(NxHalfPi);
	assert(capsuleDesc.isValid());
	actorDesc.shapes.pushBack(&capsuleDesc);

	if (density)
	{
		actorDesc.body		= &bodyDesc;
		actorDesc.density	= density;
	}
	else
	{
		actorDesc.body		= NULL;
	}

	actorDesc.globalPose.t = pos;
	assert(actorDesc.isValid());
	NxActor* pActor = gScene->createActor(actorDesc);
	pActor->setLinearDamping(10);
	assert(pActor);

	return pActor;
}

NxActor* World::MyCreateGroundPlane()
{
    // Create a plane with default descriptor
    NxPlaneShapeDesc planeDesc;
    NxActorDesc actorDesc;
    actorDesc.shapes.pushBack(&planeDesc);
    return gScene->createActor(actorDesc);
}

NxActor* World::MyCreateConvexMesh()
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	NxVec3 boxDim(1.5, 0.75, 1.6);
	// Pyramid
	NxVec3 verts[8] = 
	{
		NxVec3(boxDim.x,      -boxDim.y,  -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y,  -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y,  boxDim.z),
		NxVec3(boxDim.x,      -boxDim.y,  boxDim.z), 
		NxVec3(boxDim.x*0.5,   boxDim.y,  -boxDim.z*0.5), 
		NxVec3(-boxDim.x*0.5,  boxDim.y,  -boxDim.z*0.5),
		NxVec3(-boxDim.x*0.5,  boxDim.y,  boxDim.z*0.5), 
		NxVec3(boxDim.x*0.5,   boxDim.y,  boxDim.z*0.5)
	};

	// Create descriptor for convex mesh
	if(!convexDesc)
	{
		convexDesc = new NxConvexMeshDesc();
		assert(convexDesc);
	}
	convexDesc->numVertices = 8;
	convexDesc->pointStrideBytes = sizeof(NxVec3);
	convexDesc->points = verts;
	convexDesc->flags = NX_CF_COMPUTE_CONVEX;

	NxConvexShapeDesc convexShapeDesc;
	convexShapeDesc.localPose.t = NxVec3(0, boxDim.y, 0);
	convexShapeDesc.userData = convexDesc;

	NxInitCooking();
	MemoryWriteBuffer buf;
	bool status = NxCookConvexMesh(*convexDesc, buf);

	NxConvexMesh *pMesh = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
	assert(pMesh);
	convexShapeDesc.meshData = pMesh;
	NxCloseCooking();

	if(pMesh)
	{
		// Save mesh in userData for drawing
		pMesh->saveToDesc(*convexDesc);

		NxActorDesc actorDesc;
		assert(convexShapeDesc.isValid());
		actorDesc.shapes.pushBack(&convexShapeDesc);
		actorDesc.body = &bodyDesc;
		actorDesc.density = 1.0f;

		actorDesc.globalPose.t = NxVec3(0.0f, 0.0f, 0.0f);
		assert(actorDesc.isValid());
		NxActor* actor = gScene->createActor(actorDesc);
		assert(actor);
		return actor;
	}

	return NULL;
}

void World::MyCreateStack()
{
	NxVec3 boxDim	= NxVec3(0.25f, 0.25f, 0.25f);
	NxVec3 pos		= NxVec3(0, 0, 0);
	NxVec3 stackDim = NxVec3(6, 3, 6);

	NxActor** stack = new NxActor*[(int)(8*stackDim.x * stackDim.y * stackDim.z)];
	assert(stack);
	NxVec3 offset   = NxVec3(boxDim.x, 0, boxDim.z) + pos;

	NxI32 count = 0;
	NxI32 i = 0;
	NxI32 j = 0;
	NxI32 k = 0;
	NxVec3 boxPos;
	for (i=-stackDim.x; i<stackDim.x; i++)
	{
		for (j=0; j<stackDim.y; j++)
		{
			for (k=-stackDim.z; k<stackDim.z; k++)
			{
				boxPos = NxVec3(i * boxDim.x * 2, j * boxDim.y * 2, k * boxDim.z * 2) + offset;
				stack[count++] = CreateBox(boxPos, boxDim, 0,10.0f);
			}
		}	
	}
}

NxActor* World::CreateBox(const NxVec3& pos, const NxVec3& boxDim, const NxReal angleY, const NxReal density)
{
	// Add a single-shape actor to the scene
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// The actor has one shape, a box
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions.set(boxDim.x,boxDim.y,boxDim.z);
	boxDesc.localPose.t = NxVec3(0,boxDim.y,0);
	
	actorDesc.shapes.pushBack(&boxDesc);

	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = NULL;
	}
	actorDesc.globalPose.t = pos;
	actorDesc.globalPose.M.rotY(angleY);

	NxActor* pActor=gScene->createActor(actorDesc);

	return pActor;
}

NxActor* World::CreateSphere(const NxVec3& pos, const NxReal radius, const NxReal density)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// Create a sphere
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius = radius;

	sphereDesc.localPose.t = NxVec3(0, radius, 0);;
	sphereDesc.skinWidth = 0.05;

	assert(sphereDesc.isValid());
	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.globalPose.t = pos;

	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	
	assert(actorDesc.isValid());

	NxActor* pActor = gScene->createActor(actorDesc);
	pActor->setLinearDamping(10);
	assert(pActor);

	return pActor;
}

NxActor* World::MyCreateTriangleMesh()
{
	NxVec3 boxDim(1.0f, 1.0f, 1.0f);
	NxVec3 verts[8] = 
	{
		NxVec3(-boxDim.x, -boxDim.y, -boxDim.z), 
		NxVec3(boxDim.x,  -boxDim.y, -boxDim.z), 
		NxVec3(-boxDim.x, boxDim.y,  -boxDim.z), 
		NxVec3(boxDim.x,  boxDim.y,  -boxDim.z),
		NxVec3(-boxDim.x, -boxDim.y, boxDim.z), 
		NxVec3(boxDim.x,  -boxDim.y, boxDim.z), 
		NxVec3(-boxDim.x, boxDim.y,  boxDim.z), 
		NxVec3(boxDim.x,  boxDim.y,  boxDim.z),
	};

	NxU32 indices[12*3] =
 		{	1,2,3,        
			0,2,1,  
			5,4,1,    
			1,4,0,    
			1,3,5,    
			3,7,5,    
			3,2,7,    
			2,6,7,    
			2,0,6,    
			4,6,0,
			7,4,5,
			7,6,4
		};

	// Create descriptor for triangle mesh
	if(!triangleMeshDesc)
	{
		triangleMeshDesc = new NxTriangleMeshDesc();
		assert(triangleMeshDesc);
	}
	triangleMeshDesc->numVertices			= 8;
	triangleMeshDesc->pointStrideBytes		= sizeof(NxVec3);
	triangleMeshDesc->points = verts;
	triangleMeshDesc->numTriangles			= 12;
	triangleMeshDesc->flags					= 0;
	triangleMeshDesc->triangles				= indices;
	triangleMeshDesc->triangleStrideBytes	= 3 * sizeof(NxU32);

	NxInitCooking();
	MemoryWriteBuffer buf;

	bool status = NxCookTriangleMesh(*triangleMeshDesc, buf);
	NxTriangleMesh* pMesh;
	if(status)
	{
		pMesh = gPhysicsSDK->createTriangleMesh(MemoryReadBuffer(buf.data));
	}
	else
	{
		assert(false);
		pMesh = NULL;
	}
	NxCloseCooking();

	NxTriangleMeshShapeDesc tmsd;
	tmsd.meshData		=pMesh;
	tmsd.userData		=triangleMeshDesc;
	tmsd.localPose.t	=NxVec3(0, boxDim.y, 0);
	tmsd.meshPagingMode =NX_MESH_PAGING_AUTO;

	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	assert(tmsd.isValid());
	actorDesc.shapes.pushBack(&tmsd);
	actorDesc.body = NULL;
	actorDesc.globalPose.t = NxVec3(3.0f, 0.0f, 0.0f);

	if(pMesh)
	{
		// Save mesh in userData for drawing
		pMesh->saveToDesc(*triangleMeshDesc);
		assert(actorDesc.isValid());
		NxActor* actor = gScene->createActor(actorDesc);
		assert(actor);

		return actor;
	}

	return NULL;
}

NxActor* World::MyCreatePrimaryMultiShape()
{
	// Create an actor which is composed of box capsule and sphere.
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// Box shape
	NxBoxShapeDesc boxShape;
	boxShape.dimensions = NxVec3(1.0f, 1.0f, 1.0f);
	boxShape.localPose.t = NxVec3(1.5f, 0.0f, 0.0f);
	assert(boxShape.isValid());
	actorDesc.shapes.pushBack(&boxShape);

	// Capsule shape
	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.radius = 0.8f;
	capsuleDesc.height = 1.0f;
	capsuleDesc.localPose.t = NxVec3(0.0f, 0.0f, 0.0f);
	// Rotate capsule 90 degree around z axis
	NxQuat quat(90.0f, NxVec3(0,0,1));
	NxMat33 m;
	m.id();
	m.fromQuat(quat);
	capsuleDesc.localPose.M = m;
	assert(capsuleDesc.isValid());
	actorDesc.shapes.pushBack(&capsuleDesc);

	// Sphere Shape
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius = 1.0f;
	sphereDesc.localPose.t = NxVec3(-1.5f, 0.0f, 0.0f);
	assert(sphereDesc.isValid());
	actorDesc.shapes.pushBack(&sphereDesc);

	actorDesc.body = &bodyDesc;
	actorDesc.density = 1.0f;
	actorDesc.globalPose.t = NxVec3(5.0f, 0.0f, 0.0f);
	assert(actorDesc.isValid());
	NxActor *pActor = gScene->createActor(actorDesc);
	assert(pActor);
	return pActor;
}


NxActor* World::MyCreateMeshMultiShape()
{
	// Create an actor which is composed of Convex and Sphere
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// Convex Shape
	NxVec3 boxDim(2.0f, 2.0f, 2.0f);

	NxVec3 verts[8] = 
	{
		NxVec3(boxDim.x,      -boxDim.y, -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y, -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y, boxDim.z),
		NxVec3(boxDim.x,      -boxDim.y, boxDim.z), 
		NxVec3(boxDim.x*0.5,  boxDim.y,  -boxDim.z*0.5), 
		NxVec3(-boxDim.x*0.5, boxDim.y,  -boxDim.z*0.5),
		NxVec3(-boxDim.x*0.5, boxDim.y,  boxDim.z*0.5), 
		NxVec3(boxDim.x*0.5,  boxDim.y,  boxDim.z*0.5)
	};

	// Create descriptor for convex mesh
	if(!convexDesc)
		convexDesc = new NxConvexMeshDesc();
	assert(convexDesc);
	convexDesc->numVertices = 8;
	convexDesc->pointStrideBytes = sizeof(NxVec3);
	convexDesc->points = verts;
	convexDesc->flags = NX_CF_COMPUTE_CONVEX;

	NxConvexShapeDesc convexShapeDesc;
	convexShapeDesc.localPose.t = NxVec3(0, boxDim.y, 0);
	convexShapeDesc.userData = convexDesc;

	// Cooking Mesh
	NxInitCooking();
	MemoryWriteBuffer buf;
	bool status = NxCookConvexMesh(*convexDesc, buf);
	NxConvexMesh* pMesh = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
	assert(pMesh);
	convexShapeDesc.meshData = pMesh;
	NxCloseCooking();

	if(pMesh)
	{
		pMesh->saveToDesc(*convexDesc);

		assert(convexShapeDesc.isValid());
		actorDesc.shapes.pushBack(&convexShapeDesc);
	}

	// Sphere Shape
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius = 1.8f;
	sphereDesc.localPose.t  = NxVec3(0.0f, 5.4f, 0.0f);
	assert(sphereDesc.isValid());
	
	//actorDesc.body = &bodyDesc;
	actorDesc.shapes.pushBack(&sphereDesc);

	NxActor* pActor = gScene->createActor(actorDesc);
	assert(pActor);
	
	return pActor;
}

NxActor* World::MyCreateDynamicActor()
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions = NxVec3(1.5f, 1.5f, 1.5f);
	actorDesc.shapes.pushBack(&boxDesc);

	actorDesc.density = 1.0f;
	actorDesc.body = &bodyDesc;
	actorDesc.globalPose.t = NxVec3(3.0f, 0.0f, 0.0f);

	NxActor* pActor = gScene->createActor(actorDesc);

	return pActor;
}

NxActor* World::MyCreateKinematicActor()
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	bodyDesc.flags |= NX_BF_KINEMATIC;

	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.radius = 1.0f;
	capsuleDesc.height = 1.5f;
	actorDesc.shapes.pushBack(&capsuleDesc);

	actorDesc.density = 1.0f;
	actorDesc.body = &bodyDesc;
	actorDesc.globalPose.t = NxVec3(0.0f, 2.0f, 0.0f);

	NxActor* pActor = gScene->createActor(actorDesc);
	assert(pActor);

	return pActor;
}

NxActor* World::CreateDownWedge(const NxVec3& pos, const NxVec3& boxDim, const NxReal density)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// Down wedge
	NxVec3 verts[6] = {NxVec3(boxDim.x,boxDim.y,-boxDim.z), NxVec3(boxDim.x,boxDim.y,boxDim.z), NxVec3(boxDim.x,-boxDim.y,0),
						NxVec3(-boxDim.x,boxDim.y,-boxDim.z), NxVec3(-boxDim.x,boxDim.y,boxDim.z), NxVec3(-boxDim.x,-boxDim.y,0)};

	// Create descriptor for convex mesh
    NxConvexMeshDesc convexDesc;
    convexDesc.numVertices			= 6;
    convexDesc.pointStrideBytes		= sizeof(NxVec3);
    convexDesc.points				= verts;
    convexDesc.flags				= NX_CF_COMPUTE_CONVEX;

    NxConvexShapeDesc convexShapeDesc;
	convexShapeDesc.localPose.t		= NxVec3(0,boxDim.y,0);

    NxInitCooking();
    if (0)
    {
        // Cooking from file
#ifdef WIN32
        bool status = NxCookConvexMesh(convexDesc, UserStream("c:\\tmp.bin", false));
        convexShapeDesc.meshData = gPhysicsSDK->createConvexMesh(UserStream("c:\\tmp.bin", true));
#endif
    }
    else
    {
        // Cooking from memory
        MemoryWriteBuffer buf;
        bool status = NxCookConvexMesh(convexDesc, buf);
        convexShapeDesc.meshData = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
		printf("cook from memory\n");
    }

    if (convexShapeDesc.meshData)
    {
        NxActorDesc actorDesc;
        actorDesc.shapes.pushBack(&convexShapeDesc);
	    if (density)
   	    {
		    actorDesc.body = &bodyDesc;
		    actorDesc.density = density;
	    }
	    else
	    {
		    actorDesc.body = NULL;
	    }
        actorDesc.globalPose.t  = pos;
		NxActor* actor = gScene->createActor(actorDesc);

		return actor;
//      gPhysicsSDK->releaseConvexMesh(*convexShapeDesc.meshData);
    }
    return NULL;
}

NxActor* World::CreateRegularPyramid(const NxVec3& pos, const NxVec3& boxDim, const NxReal density)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// Pyramid
	NxVec3 verts[5] = {	NxVec3(boxDim.x,-boxDim.y,-boxDim.z), NxVec3(-boxDim.x,-boxDim.y,-boxDim.z), NxVec3(-boxDim.x,-boxDim.y,boxDim.z), 
						NxVec3(boxDim.x,-boxDim.y,boxDim.z), NxVec3(0,boxDim.y,0) };

    // Create descriptor for convex mesh
    NxConvexMeshDesc convexDesc;
    convexDesc.numVertices			= 5;
    convexDesc.pointStrideBytes		= sizeof(NxVec3);
    convexDesc.points				= verts;
    convexDesc.flags				= NX_CF_COMPUTE_CONVEX;

    NxConvexShapeDesc convexShapeDesc;
	convexShapeDesc.localPose.t		= NxVec3(0,boxDim.y,0);

    NxInitCooking();
    if (0)
    {
        // Cooking from file
#ifdef WIN32
        bool status = NxCookConvexMesh(convexDesc, UserStream("c:\\tmp.bin", false));
        convexShapeDesc.meshData = gPhysicsSDK->createConvexMesh(UserStream("c:\\tmp.bin", true));
#endif
    }
    else
    {
        // Cooking from memory
        MemoryWriteBuffer buf;
        bool status = NxCookConvexMesh(convexDesc, buf);
        convexShapeDesc.meshData = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
    }

	//
	// Please note about the created Convex Mesh, user needs to release it when no one uses it to save memory. It can be detected
	// by API "meshData->getReferenceCount() == 0". And, the release API is "gPhysicsSDK->releaseConvexMesh(*convexShapeDesc.meshData);"
	//

    if (convexShapeDesc.meshData)
    {
        NxActorDesc actorDesc;
        actorDesc.shapes.pushBack(&convexShapeDesc);
        actorDesc.body			= &bodyDesc;
        actorDesc.density		= density;
        actorDesc.globalPose.t  = pos;
		NxActor* actor = gScene->createActor(actorDesc);

		return actor;
    }

    return NULL;
}

NxActor* World::CreateCompoundCapsule(const NxVec3& pos0, const NxReal height0, const NxReal radius0, const NxReal height1, const NxReal radius1)
{
	// Add a single-shape actor to the scene
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// The actor has two shapes (capsules)
	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.height = height0;
	capsuleDesc.radius = radius0;
	capsuleDesc.localPose.t = NxVec3(0,radius0+(NxReal)0.5*height0,0);
	NxMat33 rot0, rot1;
	rot0.rotX(-0.4);
	rot1.rotZ(-0.3);
	rot0 = rot0*rot1;
	capsuleDesc.localPose.M = rot0;
	actorDesc.shapes.pushBack(&capsuleDesc);

	NxCapsuleShapeDesc capsuleDesc1;
	capsuleDesc1.height = height1;
	capsuleDesc1.radius = radius1;
	capsuleDesc1.localPose.t = NxVec3(0.33,3.15,-0.4);
	rot0.rotY(-0.04);
	capsuleDesc1.localPose.M = rot0;
	actorDesc.shapes.pushBack(&capsuleDesc1);

	actorDesc.body = &bodyDesc;
	actorDesc.density = 1.0;

	actorDesc.globalPose.t = pos0;
	return gScene->createActor(actorDesc);
}

NxActor* World::CreateOrientedCapsule(const NxMat34& globalPose, const NxReal height, const NxReal radius, const NxReal density)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.height = height;
	capsuleDesc.radius = radius;
	capsuleDesc.localPose.t = NxVec3(0, radius+(NxReal)0.5*height, 0);
	actorDesc.shapes.pushBack(&capsuleDesc);

	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = NULL;
	}
	actorDesc.globalPose = globalPose;

	return gScene->createActor(actorDesc);
}