#ifndef QUADROPHYSX_SIMULATIONTASK_H
#define QUADROPHYSX_SIMULATIONTASK_H


#include <PxPhysicsAPI.h>
#include "Strategy.h"
#include "RobotConfig.h"
#include "Robot.h"

namespace quadrophysx {

    class SimulationTask {
    public:
        enum Status {
            NOT_STARTED, RUNNING, FINISHED
        };

    private:
        int _id;
        size_t _epoch = 0;
        size_t _epochs;
        float _elapsedTime;
        physx::PxPhysics *_physics;
        quadrophysx::SimulationTask::Status _status = NOT_STARTED;
        physx::PxSceneDesc *_sceneDesc;
        quadrophysx::Strategy *_strategy;
        physx::PxScene *_scene;
        quadrophysx::Robot *_robot;
        quadrophysx::RobotConfig *_robotConfig;
    public:

        SimulationTask(int id, size_t epochs, float elapsedTime, quadrophysx::Strategy *strategy, quadrophysx::RobotConfig *robotConfig);

        void setup(physx::PxPhysics *physics, physx::PxSceneDesc *sceneDesc);

        size_t getId() const { return _id; }

        Status onStatusCheck();

        size_t onFirstRun();

        size_t onBeforeRun();

        size_t onRun();

        size_t onAfterRun();

        size_t onFinish();

        double getResult() const;

        quadrophysx::Strategy *getStrategy() const;
    };

}

#endif //QUADROPHYSX_SIMULATIONTASK_H
