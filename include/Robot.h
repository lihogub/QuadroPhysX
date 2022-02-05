#ifndef QUADROPHYSX_ROBOT_H
#define QUADROPHYSX_ROBOT_H


#include <PxPhysicsAPI.h>
#include "RobotConfig.h"
#include "RobotState.h"


namespace quadrophysx {

    class Robot {
    private:
        physx::PxArticulationReducedCoordinate*      _articulation;
        physx::PxArticulationLink*                   _rootLink;
        physx::PxArticulationLink*                   _links[4][3];
        physx::PxArticulationJointReducedCoordinate* _joints[4][3];
        physx::PxU32                                 _jointsIndexes[4][3];
        physx::PxArticulationCache*                  _cache;
        quadrophysx::RobotState*                     _robotState;
        quadrophysx::RobotConfig                     _robotConfig;
        void initJointIndexes();
    public:
        Robot(physx::PxPhysics* physics, physx::PxScene* scene, quadrophysx::RobotConfig& config);
        physx::PxArticulationReducedCoordinate* getArticulation() { return _articulation; }
        const quadrophysx::RobotState* getRobotState() { return _robotState; };
        physx::PxArticulationCache* getCache() { return _cache; };
        void updateRobotState();
        void setTargetPositionAndVelocity(int legIndex, int jointIndex, float position, float velocity);
    };

}


#endif //QUADROPHYSX_ROBOT_H
