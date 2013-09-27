#include "stdafx.h"
#include "MyPhysx.h"

NxRevoluteJoint* World::CreateRevoluteJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis, bool isMotor)
{
	NxRevoluteJointDesc revDesc;

	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	//revDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	if (!isMotor)
	{
		//revDesc.flags |= NX_RJF_SPRING_ENABLED;
		//NxSpringDesc springDesc;
		//springDesc.spring = 5000;
		//springDesc.damper = 50;
		//springDesc.targetValue = 1.5*NxPi;
		//revDesc.spring = springDesc;
	}
	else	// motor enabled
	{
		revDesc.flags |= NX_RJF_MOTOR_ENABLED;
		NxMotorDesc motorDesc;
		motorDesc.setToDefault();
		motorDesc.velTarget = 3;
		motorDesc.maxForce = 10;
		motorDesc.freeSpin = true;
		revDesc.motor = motorDesc;
	}

	revDesc.projectionMode = NX_JPM_POINT_MINDIST;
	revDesc.projectionDistance = 1.0f;
	revDesc.projectionAngle = 0.0872f;

	

	return (NxRevoluteJoint*)gScene->createJoint(revDesc);
}

NxFixedJoint* World::CreateFixedJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis)
{
	NxFixedJointDesc fixedDesc;

	fixedDesc.actor[0] = a0;
	fixedDesc.actor[1] = a1;
	fixedDesc.setGlobalAnchor(globalAnchor);
	fixedDesc.setGlobalAxis(globalAxis);

	return (NxFixedJoint*)gScene->createJoint(fixedDesc);
}

NxSphericalJoint* World::CreateSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	sphericalDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
	sphericalDesc.swingLimit.value = 0.3*NxPi;

	sphericalDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
	sphericalDesc.twistLimit.low.value = -0.05*NxPi;
	sphericalDesc.twistLimit.high.value = 0.05*NxPi;

	return (NxSphericalJoint*)gScene->createJoint(sphericalDesc);
}

NxPrismaticJoint* World::CreatePrismaticJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis)
{
	NxPrismaticJointDesc prismaticDesc;
	prismaticDesc.actor[0] = a0;
	prismaticDesc.actor[1] = a1;
	prismaticDesc.setGlobalAnchor(globalAnchor);		// 0 7 0
	prismaticDesc.setGlobalAxis(globalAxis);			// 0 1 0

	NxJoint* joint = gScene->createJoint(prismaticDesc);

	joint->setLimitPoint(globalAnchor);
	joint->addLimitPlane(-globalAxis, globalAnchor + 1*globalAxis, 0.5f);
	joint->addLimitPlane(globalAxis, globalAnchor - 1*globalAxis, 0.5f);

	return (NxPrismaticJoint*)joint;
}

NxCylindricalJoint* World::CreateCylindricalJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis)
{
	NxCylindricalJointDesc cylDesc;
	cylDesc.actor[0] = a0;
	cylDesc.actor[1] = a1;
	cylDesc.setGlobalAnchor(globalAnchor);
	cylDesc.setGlobalAxis(globalAxis);

	NxJoint* joint = gScene->createJoint(cylDesc);

	joint->setLimitPoint(globalAnchor);

	// Add up-down limiting planes
	joint->addLimitPlane(-globalAxis, globalAnchor + 1*globalAxis);
	joint->addLimitPlane(globalAxis, globalAnchor - 1*globalAxis);

	return (NxCylindricalJoint*)joint;
}

NxPointOnLineJoint* World::CreatePointOnLineJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis)
{
	NxPointOnLineJointDesc polDesc;
	polDesc.actor[0] = a0;
	polDesc.actor[1] = a1;
	polDesc.setGlobalAnchor(globalAnchor);
	polDesc.setGlobalAxis(globalAxis);
	polDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	return (NxPointOnLineJoint*)gScene->createJoint(polDesc);
}

NxPointInPlaneJoint* World::CreatePointInPlaneJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis)
{
	NxPointInPlaneJointDesc pipDesc;
	pipDesc.actor[0] = a0;
	pipDesc.actor[1] = a1;
	pipDesc.setGlobalAnchor(globalAnchor);
	pipDesc.setGlobalAxis(globalAxis);
	pipDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	return (NxPointInPlaneJoint*)gScene->createJoint(pipDesc);
}

NxPulleyJoint* World::CreatePulleyJoint(NxActor* a0, NxActor* a1, const NxVec3& pulley0, const NxVec3& pulley1, 
										const NxVec3& globalAxis, NxReal distance, NxReal ratio, const NxMotorDesc& motorDesc)
{
	NxPulleyJointDesc pulleyDesc;
	pulleyDesc.actor[0] = a0;
	pulleyDesc.actor[1] = a1;
	pulleyDesc.localAnchor[0] = NxVec3(0,2,0);	
	pulleyDesc.localAnchor[1] = NxVec3(0,2,0);
	pulleyDesc.setGlobalAxis(globalAxis);

	pulleyDesc.pulley[0] = pulley0;			// suspension points of two bodies in world space.
	pulleyDesc.pulley[1] = pulley1;			// suspension points of two bodies in world space.
	pulleyDesc.distance = distance;			// the rest length of the rope connecting the two objects.
											// The distance is computed as ||(pulley0 - anchor0)|| +  ||(pulley1 - anchor1)|| * ratio.
	pulleyDesc.ratio = ratio;				// transmission ratio
	pulleyDesc.flags = NX_PJF_IS_RIGID; //| NX_PJF_MOTOR_ENABLED;		// transmission ratio
	pulleyDesc.motor = motorDesc;
	pulleyDesc.stiffness = 0.1;				// how stiff the constraint is, between 0 and 1 (stiffest)

//	pulleyDesc.projectionMode = NX_JPM_NONE;
//	pulleyDesc.projectionMode = NX_JPM_POINT_MINDIST;

	pulleyDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	return (NxPulleyJoint*)gScene->createJoint(pulleyDesc);
}

NxDistanceJoint* World::CreateDistanceJoint(NxActor* a0, NxActor* a1, const NxVec3& anchor0, const NxVec3& anchor1, const NxVec3& globalAxis)
{
	NxDistanceJointDesc distanceDesc;
	distanceDesc.actor[0] = a0;
	distanceDesc.actor[1] = a1;
	distanceDesc.localAnchor[0] = anchor0;
	distanceDesc.localAnchor[1] = anchor1;
	distanceDesc.setGlobalAxis(globalAxis);

	NxVec3 dist = a1->getGlobalPose()*anchor1 - a0->getGlobalPose()*anchor0;
	printf("dist is %f %f %f\n",dist.x, dist.y, dist.z);
	distanceDesc.maxDistance = dist.magnitude()*1.5f;
	distanceDesc.minDistance = dist.magnitude()*0.1f;

	printf("maxDistance %f, minDistance %f\n", distanceDesc.maxDistance, distanceDesc.minDistance);

	NxSpringDesc spring;
	spring.spring = 1000;		// µ¯ÐÔÏµÊý
	spring.damper = 0.5;
	distanceDesc.spring = spring;
	distanceDesc.flags = (NX_DJF_MIN_DISTANCE_ENABLED | NX_DJF_MAX_DISTANCE_ENABLED);
	distanceDesc.flags |= NX_DJF_SPRING_ENABLED;
	distanceDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	return (NxDistanceJoint*)gScene->createJoint(distanceDesc);
}

NxD6Joint* World::CreateD6Joint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxD6JointDesc d6Desc;
	d6Desc.actor[0] = a0;
	d6Desc.actor[1] = a1;
	d6Desc.setGlobalAnchor(globalAnchor);
	d6Desc.setGlobalAxis(globalAxis);

	d6Desc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

	d6Desc.xMotion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	d6Desc.zMotion = NX_D6JOINT_MOTION_LOCKED;

	d6Desc.projectionMode = NX_JPM_NONE;

	return (NxD6Joint*)gScene->createJoint(d6Desc);

}

void World::ReconfigureD6Joint(NxD6Joint* d6Joint, NxActor* a0, NxActor* a1)
{
	NxD6JointDesc oldD6Desc, d6Desc;
	NxVec3 localAnchor[2], localAxis[2], localNormal[2], localBinormal[2];

	d6Joint->saveToDesc(oldD6Desc);

	localAnchor[0] = oldD6Desc.localAnchor[0];
	localAnchor[1] = oldD6Desc.localAnchor[1];

	localAxis[0] = oldD6Desc.localAxis[0];
	localNormal[0] = oldD6Desc.localNormal[0];
	localBinormal[0] = oldD6Desc.localNormal[0].cross(localAxis[0]);		// ???????

	localAxis[1] = oldD6Desc.localAxis[1];
	localNormal[1] = oldD6Desc.localNormal[1];
	localBinormal[1] = oldD6Desc.localNormal[0].cross(localAxis[1]);		// ???????

	switch (gJointType)
	{
		case 0:  // Fixed Joint 
		{  
			// Coming from spherical joint, so reset actor #1
			NxMat33 orient;
			orient.id();
			a1->raiseBodyFlag(NX_BF_KINEMATIC);
			a1->setGlobalOrientation(orient);
			a1->setGlobalPosition(NxVec3(0,3,0));
			a1->clearBodyFlag(NX_BF_KINEMATIC);

			d6Desc.actor[0] = a0;
			d6Desc.actor[1] = a1;

			// Reset anchor and axis
			NxVec3 globalAnchor = NxVec3(0,5,0);
			NxVec3 globalAxis = NxVec3(0,0,-1);

			d6Desc.setGlobalAnchor(globalAnchor);
			d6Desc.setGlobalAxis(globalAxis);

			d6Desc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

			d6Desc.xMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.zMotion = NX_D6JOINT_MOTION_LOCKED;
		}
		break;
		case 1:  // Revolute Joint 
		{  
			d6Desc.actor[0] = a0;
			d6Desc.actor[1] = a1;
			d6Desc.localAnchor[0] = localAnchor[0];
			d6Desc.localAnchor[1] = localAnchor[1];
			d6Desc.localAxis[0] = localAxis[0];
			d6Desc.localNormal[0] = localNormal[0];
			d6Desc.localAxis[1] = localAxis[1];
			d6Desc.localNormal[1] = localNormal[1];

			d6Desc.twistMotion = NX_D6JOINT_MOTION_FREE;
			d6Desc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

			d6Desc.xMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.zMotion = NX_D6JOINT_MOTION_LOCKED;
		}
		break;

		case 2:  // Spherical Joint 
		{  
			d6Desc.actor[0] = a0;
			d6Desc.actor[1] = a1;

			d6Desc.localAnchor[0] = localAnchor[0];
			d6Desc.localAnchor[1] = localAnchor[1];
			d6Desc.localAxis[0] = localBinormal[0];
			d6Desc.localNormal[0] = localNormal[0];
			d6Desc.localAxis[1] = localBinormal[1];
			d6Desc.localNormal[1] = localNormal[1];

			d6Desc.twistMotion = NX_D6JOINT_MOTION_FREE;
			d6Desc.swing1Motion = NX_D6JOINT_MOTION_FREE;
			d6Desc.swing2Motion = NX_D6JOINT_MOTION_FREE;

			d6Desc.xMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
			d6Desc.zMotion = NX_D6JOINT_MOTION_LOCKED;
		}
		break;
	};

	d6Desc.projectionMode = NX_JPM_NONE;

	// Set joint motion display values
	gJointMotion[3] = d6Desc.twistMotion;
	gJointMotion[4] = d6Desc.swing1Motion;
	gJointMotion[5] = d6Desc.swing2Motion;

	gJointMotion[0] = d6Desc.xMotion;
	gJointMotion[1] = d6Desc.yMotion;
	gJointMotion[2] = d6Desc.zMotion;

	char ds[512];

	// Set joint type in HUD
	sprintf(ds, "JOINT TYPE: %s", gJointTypeString[gJointType]);
	hud.SetDisplayString(2, ds, 0.015f, 0.92f);	

	// Set rotation motions in HUD
    sprintf(ds, "   Axis: %s", gJointMotionString[gJointMotion[3]]);
	hud.SetDisplayString(4, ds, 0.015f, 0.82f); 
    sprintf(ds, "   Normal: %s", gJointMotionString[gJointMotion[4]]);
	hud.SetDisplayString(5, ds, 0.015f, 0.77f); 
    sprintf(ds, "   Binormal: %s", gJointMotionString[gJointMotion[5]]);
	hud.SetDisplayString(6, ds, 0.015f, 0.72f); 

	// Set translation motions in HUD
    sprintf(ds, "   Axis: %s", gJointMotionString[gJointMotion[0]]);
	hud.SetDisplayString(8, ds, 0.015f, 0.62f); 
    sprintf(ds, "   Normal: %s", gJointMotionString[gJointMotion[1]]);
	hud.SetDisplayString(9, ds, 0.015f, 0.57f); 
    sprintf(ds, "   Binormal: %s", gJointMotionString[gJointMotion[2]]);
	hud.SetDisplayString(10, ds, 0.015f, 0.52f);

	d6Joint->loadFromDesc(d6Desc);
}

NxSphericalJoint* World::CreateRopeSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	sphericalDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
	sphericalDesc.twistLimit.low.value = -(NxReal)0.1*NxPi;
	sphericalDesc.twistLimit.high.value = (NxReal)0.1*NxPi;

	sphericalDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
	NxSpringDesc ts;
	ts.spring = 500;
	ts.damper = 0.5;
	ts.targetValue = 0;
	sphericalDesc.twistSpring = ts;

	sphericalDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
	sphericalDesc.swingLimit.value = (NxReal)0.3*NxPi;

	sphericalDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
	NxSpringDesc ss;
	ss.spring = 500;
	ss.damper = 0.5;
	ss.targetValue = 0;
	sphericalDesc.swingSpring = ss;

	return (NxSphericalJoint*)gScene->createJoint(sphericalDesc);
}