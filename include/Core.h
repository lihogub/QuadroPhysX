#ifndef QUADROPHYSX_CORE_H
#define QUADROPHYSX_CORE_H

#include <queue>
#include <chrono>
#include <mutex>
#include "PxPhysicsAPI.h"
#include "RobotConfig.h"
#include "task/SimulationTask.h"

namespace quadrophysx {

    class Core {
    private:
        size_t _coreId;
        std::mutex* _coreMutex;
        std::condition_variable _coreCv;
        physx::PxFoundation *_foundation;
        physx::PxPhysics *_physics;
        physx::PxSceneDesc *_sceneDesc;
        std::queue<quadrophysx::SimulationTask*> _taskQueue;
        std::queue<quadrophysx::SimulationTask*> _finishedTaskQueue;
        std::queue<quadrophysx::SimulationTask*> _notStartedTaskQueue;
        size_t _sceneCount = 0;
        size_t _maxSceneCount;
        void tick() { ++_tick; };
        std::atomic<size_t> _tick;
    public:
        Core(size_t coreId, std::mutex *coreMutex, physx::PxFoundation *foundation, size_t numThreads, size_t maxSceneCount);
        void addTask(quadrophysx::SimulationTask *task);
        void spin(size_t iterations);
        size_t getQueuedTasksCount();
        std::queue<quadrophysx::SimulationTask *> getFinishedTasksQueue();
        size_t getFinishedTasksQueueSize();
        void clearTaskQueue();
        size_t getTicks() { return _tick; };
    };

}

#endif //QUADROPHYSX_CORE_H
