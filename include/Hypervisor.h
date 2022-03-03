#ifndef QUADROPHYSX_HYPERVISOR_H
#define QUADROPHYSX_HYPERVISOR_H

#include <thread>
#include <mutex>
#include "Core.h"
#include "task/SimulationTask.h"
#include <queue>

namespace quadrophysx {

    class Hypervisor {
    private:
        physx::PxDefaultAllocator *_defaultAllocatorCallback;
        physx::PxDefaultErrorCallback *_defaultErrorCallback;
        physx::PxFoundation *_foundation;
        quadrophysx::Core **_cores;
        std::thread **_threads;
        std::mutex **_coreMutexes;
        size_t _coreCount;
        size_t _threadsPerCore;
        size_t _scenesPerCore;
        std::mutex _finishedTasksMutex;
        std::queue<SimulationTask *> _scheduledTasks;
        std::queue<SimulationTask *> _finishedTasks;
        quadrophysx::RobotConfig *_robotConfig;

        void spinCore(size_t coreId);

        void scheduleTask(quadrophysx::SimulationTask *task);

    public:
        Hypervisor(size_t coreCount, size_t threadsPerCore, size_t scenesPerCore);

        void submitTask(quadrophysx::SimulationTask *task);

        std::vector<quadrophysx::SimulationTask *> fetchTasks();

        void spin(size_t millis);

        void run();

        size_t getTicks();

        void addConfiguration(quadrophysx::RobotConfig &robotConfig) { _robotConfig = &robotConfig; }

        void printCores();

        size_t getFinishedTasksQueueSize();

        std::queue<SimulationTask *> getFinishedTasksQueue();

        void clearQueue();
    };

}

#endif //QUADROPHYSX_HYPERVISOR_H
