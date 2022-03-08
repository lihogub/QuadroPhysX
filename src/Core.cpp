#include <chrono>
#include "Core.h"


quadrophysx::Core::Core(size_t coreId, std::mutex *coreMutex, physx::PxFoundation *foundation, size_t numThreads,
                        size_t maxSceneCount)
        : _maxSceneCount(maxSceneCount), _coreMutex(coreMutex), _coreId(coreId) {
    _foundation = foundation;
    physx::PxTolerancesScale mToleranceScale;
    mToleranceScale.length = 100;
    mToleranceScale.speed = 981;

    physx::PxPvd *mPvd;

#ifndef RELEASE
    mPvd = PxCreatePvd(*_foundation);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    mPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
#endif
    _physics = PxCreatePhysics(PX_PHYSICS_VERSION, *_foundation, mToleranceScale, false, mPvd);
    PxInitExtensions(*_physics, mPvd);

    _sceneDesc = new physx::PxSceneDesc(_physics->getTolerancesScale());
    _sceneDesc->gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    _sceneDesc->cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(numThreads);
    _sceneDesc->filterShader = physx::PxDefaultSimulationFilterShader;
}

void quadrophysx::Core::spin(size_t iterations) {
    quadrophysx::SimulationTask::Status status;
    quadrophysx::SimulationTask *task;
    int iteration = 0;
    while (iteration < iterations) {
        size_t ticks = 0;
        if (!_taskQueue.empty()) {
            task = _taskQueue.front();
            _taskQueue.pop();

            status = task->onStatusCheck();
            switch (status) {
                case SimulationTask::Status::RUNNING: {
                    ticks += task->onBeforeRun();
                    ticks += task->onRun();
                    ticks += task->onAfterRun();
                    _taskQueue.push(task);
                    break;
                }
                case SimulationTask::Status::FINISHED: {
                    ticks += task->onFinish();
                    _finishedTaskQueue.push(task);
                    --_sceneCount;
                    break;
                }
                case SimulationTask::Status::NOT_STARTED: {
                    ticks += task->onFirstRun();
                    _taskQueue.push(task);
                    break;
                }
            }
        }

        if (_sceneCount < _maxSceneCount) {
            std::unique_lock<std::mutex> lock(*_coreMutex);
            while (_sceneCount < _maxSceneCount && !_notStartedTaskQueue.empty()) {
                task = _notStartedTaskQueue.front();
                _notStartedTaskQueue.pop();
                ++_sceneCount;
                _taskQueue.push(task);
            }
        }

        if (_sceneCount == 0) {
            std::unique_lock<std::mutex> lock(*_coreMutex);
//            std::cout << "Sleeping: " << _coreId << std::endl;
            _coreCv.wait(lock);
//            std::cout << "Wake up: " << _coreId << std::endl;
        }
        _tick += ticks;
        ++iteration;
    }
}

void quadrophysx::Core::clearTaskQueue() {
    std::unique_lock<std::mutex> lock(*_coreMutex);
    while (!_finishedTaskQueue.empty()) {
        _finishedTaskQueue.pop();
    }
}

size_t quadrophysx::Core::getFinishedTasksQueueSize() {
    std::unique_lock<std::mutex> lock(*_coreMutex);
    return _finishedTaskQueue.size();
}

size_t quadrophysx::Core::getQueuedTasksCount() {
    std::unique_lock<std::mutex> lock(*_coreMutex);
    return _taskQueue.size() + _notStartedTaskQueue.size();
}

void quadrophysx::Core::addTask(quadrophysx::SimulationTask *task) {
    task->setup(_physics, _sceneDesc);
    std::unique_lock<std::mutex> lock(*_coreMutex);
    _notStartedTaskQueue.push(task);
    _coreCv.notify_one();
}

std::queue<quadrophysx::SimulationTask *> quadrophysx::Core::getFinishedTasksQueue() {
    std::unique_lock<std::mutex> lock(*_coreMutex);
    return _finishedTaskQueue;
}
