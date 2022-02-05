#include "Robot.h"
#include <PxPhysicsAPI.h>
#include <iostream>

quadrophysx::Robot::Robot(physx::PxPhysics *physics, physx::PxScene *scene, quadrophysx::RobotConfig &config) {
    _articulation = physics->createArticulationReducedCoordinate();
    physx::PxMaterial *mMaterial = physics->createMaterial(0.5f, 0.5f, 0.5f);

    _robotConfig = config;

    /* compute positions */
    float h = 50;

    physx::PxTransform rootTransform = physx::PxTransform(physx::PxIdentity)
            .transform(
                    physx::PxTransform(0, h, 0)
            )
            .transform(
                    physx::PxTransform(physx::PxQuat(0, 0, 0, 1).getNormalized())
            );

    float w0 = _robotConfig.rootLink.size.z;
    float h0 = _robotConfig.rootLink.size.y;
    float l0 = _robotConfig.rootLink.size.x;
    float d0 = _robotConfig.rootLink.density;

    /* root link */
    _rootLink = _articulation->createLink(nullptr, rootTransform);
    physx::PxRigidActorExt::createExclusiveShape(*_rootLink, physx::PxBoxGeometry(l0, h0, w0), *mMaterial);
    physx::PxRigidBodyExt::updateMassAndInertia(*_rootLink, d0);

    for (int legIndex = 0; legIndex < 4; legIndex++) {

        float p1 = _robotConfig.joints[legIndex][0].position;
        float ll1 = _robotConfig.joints[legIndex][0].limitLower;
        float lu1 = _robotConfig.joints[legIndex][0].limitUpper;
        float st1 = _robotConfig.joints[legIndex][0].stiffness;
        float damp1 = _robotConfig.joints[legIndex][1].damping;
        float mf1 = _robotConfig.joints[legIndex][0].maxForce;
        float w1 = _robotConfig.links[legIndex][0].size.z;
        float h1 = _robotConfig.links[legIndex][0].size.y;
        float l1 = _robotConfig.links[legIndex][0].size.x;
        float d1 = _robotConfig.links[legIndex][0].density;

        float p2 = _robotConfig.joints[legIndex][1].position;
        float ll2 = _robotConfig.joints[legIndex][1].limitLower;
        float lu2 = _robotConfig.joints[legIndex][1].limitUpper;
        float st2 = _robotConfig.joints[legIndex][1].stiffness;
        float damp2 = _robotConfig.joints[legIndex][1].damping;
        float mf2 = _robotConfig.joints[legIndex][1].maxForce;
        float w2 = _robotConfig.links[legIndex][1].size.z;
        float h2 = _robotConfig.links[legIndex][1].size.y;
        float l2 = _robotConfig.links[legIndex][1].size.x;
        float d2 = _robotConfig.links[legIndex][1].density;

        float p3 = _robotConfig.joints[legIndex][2].position;
        float ll3 = _robotConfig.joints[legIndex][2].limitLower;
        float lu3 = _robotConfig.joints[legIndex][2].limitUpper;
        float st3 = _robotConfig.joints[legIndex][2].stiffness;
        float damp3 = _robotConfig.joints[legIndex][2].damping;
        float mf3 = _robotConfig.joints[legIndex][2].maxForce;
        float w3 = _robotConfig.links[legIndex][2].size.z;
        float h3 = _robotConfig.links[legIndex][2].size.y;
        float l3 = _robotConfig.links[legIndex][2].size.x;
        float d3 = _robotConfig.links[legIndex][2].density;

        if (legIndex % 2) {
            p1 *= -1;
            p2 *= -1;
            p3 *= -1;
        }

        float x1 = (l0 + l1) - 2 * (l0 + l1) * (legIndex / 2);
        float y1 = 0;
        float z1 = w0 / 2 - w0 * (legIndex % 2);
        float rotate = physx::PxPi * (legIndex % 2);

        physx::PxTransform link00Transform = rootTransform
                .transform(
                        physx::PxTransform(physx::PxVec3(x1, y1, z1))
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(rotate, physx::PxVec3(0, 1, 0)).getNormalized())
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(p1, physx::PxVec3(1, 0, 0)).getNormalized())
                )
                .transform(
                        physx::PxTransform(0, 0, w1 - 5)
                );

        physx::PxTransform joint00ParentTransform = physx::PxTransform(physx::PxIdentity)
                .transform(
                        physx::PxTransform(physx::PxVec3(x1, y1, z1))
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(rotate, physx::PxVec3(0, 1, 0)).getNormalized())
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(p1, physx::PxVec3(1, 0, 0)).getNormalized())
                );

        physx::PxTransform joint00ChildTransform = physx::PxTransform(physx::PxIdentity)
                .transform(
                        physx::PxTransform(physx::PxVec3(0, 0, -w1 + 5))
                );

        physx::PxTransform link01Transform = link00Transform
                .transform(
                        physx::PxTransform(physx::PxVec3(0, 0, w1 + l2))
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(0, 1, 0, -1).getNormalized())
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(p2, physx::PxVec3(1, 0, 0)).getNormalized())
                )
                .transform(
                        physx::PxTransform(physx::PxVec3(0, -h2 + 5, 0))
                );

        physx::PxTransform joint01ParentTransform = physx::PxTransform(physx::PxIdentity)
                .transform(
                        physx::PxTransform(physx::PxVec3(0, 0, w1 + l2))
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(0, 1, 0, -1).getNormalized())
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(p2, physx::PxVec3(1, 0, 0)).getNormalized())
                );

        physx::PxTransform joint01ChildTransform = physx::PxTransform(physx::PxIdentity)
                .transform(
                        physx::PxTransform(physx::PxVec3(0, h2 - 5, 0))
                );

        physx::PxTransform link02Transform = link01Transform
                .transform(
                        physx::PxTransform(physx::PxVec3(l2 + l3, -h2 + 5, 0))
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(p3, physx::PxVec3(1, 0, 0)).getNormalized())
                )
                .transform(
                        physx::PxTransform(physx::PxVec3(0, -h3 + 5, 0))
                );

        physx::PxTransform joint02ParentTransform = physx::PxTransform(physx::PxIdentity)
                .transform(
                        physx::PxTransform(physx::PxVec3(l2 + l3, -h2 + 5, 0))
                )
                .transform(
                        physx::PxTransform(physx::PxQuat(p3, physx::PxVec3(1, 0, 0)).getNormalized())
                );

        physx::PxTransform joint02ChildTransform = physx::PxTransform(physx::PxIdentity)
                .transform(
                        physx::PxTransform(physx::PxVec3(0, h3 - 5, 0))
                );

        _links[legIndex][0] = _articulation->createLink(_rootLink, link00Transform);
        physx::PxRigidActorExt::createExclusiveShape(*_links[legIndex][0], physx::PxBoxGeometry(l1, h1, w1),
                                                     *mMaterial);
        physx::PxRigidBodyExt::updateMassAndInertia(*_links[legIndex][0], d1);

        _links[legIndex][1] = _articulation->createLink(_links[legIndex][0], link01Transform);
        physx::PxRigidActorExt::createExclusiveShape(*_links[legIndex][1], physx::PxBoxGeometry(l2, h2, w2),
                                                     *mMaterial);
        physx::PxRigidBodyExt::updateMassAndInertia(*_links[legIndex][1], d2);

        _links[legIndex][2] = _articulation->createLink(_links[legIndex][1], link02Transform);
        physx::PxRigidActorExt::createExclusiveShape(*_links[legIndex][2], physx::PxBoxGeometry(l3, h3, w3),
                                                     *mMaterial);
        physx::PxRigidBodyExt::updateMassAndInertia(*_links[legIndex][2], d3);

        _joints[legIndex][0] = static_cast<physx::PxArticulationJointReducedCoordinate *>(_links[legIndex][0]->getInboundJoint());
        _joints[legIndex][0]->setParentPose(joint00ParentTransform);
        _joints[legIndex][0]->setChildPose(joint00ChildTransform);
        _joints[legIndex][0]->setJointType(physx::PxArticulationJointType::eREVOLUTE);
        _joints[legIndex][0]->setMotion(physx::PxArticulationAxis::eTWIST, physx::PxArticulationMotion::eLIMITED);
        _joints[legIndex][0]->setLimit(physx::PxArticulationAxis::eTWIST, ll1, lu1);
        _joints[legIndex][0]->setDrive(physx::PxArticulationAxis::eTWIST, st1, damp1, mf1);

        _joints[legIndex][1] = static_cast<physx::PxArticulationJointReducedCoordinate *>(_links[legIndex][1]->getInboundJoint());
        _joints[legIndex][1]->setParentPose(joint01ParentTransform);
        _joints[legIndex][1]->setChildPose(joint01ChildTransform);
        _joints[legIndex][1]->setJointType(physx::PxArticulationJointType::eREVOLUTE);
        _joints[legIndex][1]->setMotion(physx::PxArticulationAxis::eTWIST, physx::PxArticulationMotion::eLIMITED);
        _joints[legIndex][1]->setLimit(physx::PxArticulationAxis::eTWIST, ll2, lu2);
        _joints[legIndex][1]->setDrive(physx::PxArticulationAxis::eTWIST, st2, damp2, mf2);
        _joints[legIndex][1]->setMaxJointVelocity(0.5);

        _joints[legIndex][2] = static_cast<physx::PxArticulationJointReducedCoordinate *>(_links[legIndex][2]->getInboundJoint());
        _joints[legIndex][2]->setParentPose(joint02ParentTransform);
        _joints[legIndex][2]->setChildPose(joint02ChildTransform);
        _joints[legIndex][2]->setJointType(physx::PxArticulationJointType::eREVOLUTE);
        _joints[legIndex][2]->setMotion(physx::PxArticulationAxis::eTWIST, physx::PxArticulationMotion::eLIMITED);
        _joints[legIndex][2]->setLimit(physx::PxArticulationAxis::eTWIST, ll3, lu3);
        _joints[legIndex][2]->setDrive(physx::PxArticulationAxis::eTWIST, st3, damp3, mf3);
    }


    scene->addArticulation(*_articulation);
    _cache = _articulation->createCache();

    _robotState = new RobotState();

    initJointIndexes();
}

void quadrophysx::Robot::updateRobotState() {
    _articulation->copyInternalStateToCache(*_cache, physx::PxArticulationCache::eALL);

    _robotState->rootLink.transform = _rootLink->getGlobalPose();
    _robotState->rootLink.velocity.linear = _cache->rootLinkData->worldLinVel;
    _robotState->rootLink.velocity.angular = _cache->rootLinkData->worldAngVel;
    _robotState->rootLink.acceleration.linear = _cache->rootLinkData->worldLinAccel;
    _robotState->rootLink.acceleration.angular = _cache->rootLinkData->worldAngAccel;


    for (int legIndex = 0; legIndex < 4; legIndex++) {
        for (int partIndex = 0; partIndex < 3; partIndex++) {
            _robotState->links[legIndex][partIndex].transform = _links[legIndex][partIndex]->getGlobalPose();

            physx::PxU32 linkIndex = _links[legIndex][partIndex]->getLinkIndex();

            physx::PxSpatialVelocity spatialVelocity = _cache->linkVelocity[legIndex];
            _robotState->links[legIndex][partIndex].velocity.linear = spatialVelocity.linear;
            _robotState->links[legIndex][partIndex].velocity.angular = spatialVelocity.angular;

            physx::PxSpatialVelocity spatialAcceleration = _cache->linkAcceleration[linkIndex];
            _robotState->links[legIndex][partIndex].acceleration.linear = spatialAcceleration.linear;
            _robotState->links[legIndex][partIndex].acceleration.angular = spatialAcceleration.angular;

            physx::PxU32 jointIndex = _jointsIndexes[legIndex][partIndex];
            _robotState->joints[legIndex][partIndex].position = _cache->jointPosition[jointIndex];
            _robotState->joints[legIndex][partIndex].velocity = _cache->jointVelocity[jointIndex];
            _robotState->joints[legIndex][partIndex].acceleration = _cache->jointAcceleration[jointIndex];
            _robotState->joints[legIndex][partIndex].force = _cache->jointForce[jointIndex];
        }
    }

}

void quadrophysx::Robot::initJointIndexes() {
    physx::PxU32 TotalLinkCount = 13;
    physx::PxU32* dofStarts = new physx::PxU32[TotalLinkCount];
    dofStarts[0] = 0;

    for(int legIndex = 0; legIndex < 4; legIndex++) {
        for (int linkIndex = 0; linkIndex < 3; linkIndex++) {
            physx::PxU32 llIndex = _links[legIndex][linkIndex]->getLinkIndex();
            physx::PxU32 dofs = _links[legIndex][linkIndex]->getInboundJointDof();
            dofStarts[llIndex] = dofs;
        }
    }

    physx::PxU32 count = 0;
    for(physx::PxU32 i = 1; i < TotalLinkCount; ++i)
    {
        physx::PxU32 dofs = dofStarts[i];
        dofStarts[i] = count;
        count += dofs;
    }

    for(int legIndex = 0; legIndex < 4; legIndex++) {
        for (int linkIndex = 0; linkIndex < 3; linkIndex++) {
            _jointsIndexes[legIndex][linkIndex] = dofStarts[_links[legIndex][linkIndex]->getLinkIndex()];
        }
    }

    delete[] dofStarts;
}

void quadrophysx::Robot::setTargetPositionAndVelocity(int legIndex, int jointIndex, float position,float velocity) {
    if (legIndex % 2) {
        position *= -1;
    }

    _joints[legIndex][jointIndex]->setDriveVelocity(physx::PxArticulationAxis::eTWIST, velocity);
    _joints[legIndex][jointIndex]->setDriveTarget(physx::PxArticulationAxis::eTWIST,  position);
}
