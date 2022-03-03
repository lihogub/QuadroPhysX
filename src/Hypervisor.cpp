#include <iostream>
#include "Hypervisor.h"

quadrophysx::Hypervisor::Hypervisor(size_t coreCount, size_t threadsPerCore, size_t scenesPerCore)
        : _coreCount(coreCount), _threadsPerCore(threadsPerCore), _scenesPerCore(scenesPerCore) {
    _cores = new quadrophysx::Core*[_coreCount];
    _coreMutexes = new std::mutex*[_coreCount];
    _threads = new std::thread*[_coreCount];
    for (int i = 0; i < _coreCount; i++) {
        _coreMutexes[i] = new std::mutex;
    }
}

void quadrophysx::Hypervisor::submitTask(quadrophysx::SimulationTask *task) {
    scheduleTask(task);
}

std::vector<quadrophysx::SimulationTask *> quadrophysx::Hypervisor::fetchTasks() {
    std::unique_lock<std::mutex> lock(_finishedTasksMutex);
    std::vector<quadrophysx::SimulationTask *> vector;

    for (int i = 0; i < _coreCount; i++) {
        std::queue<quadrophysx::SimulationTask *> tmpQueue = _cores[i]->getFinishedTasksQueue();
        while (!tmpQueue.empty()) {
            vector.push_back(tmpQueue.front());
            tmpQueue.pop();
        }
    }

    return vector;
}

void quadrophysx::Hypervisor::spin(size_t millis) {
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

void quadrophysx::Hypervisor::run() {
    _defaultAllocatorCallback = new physx::PxDefaultAllocator;
    _defaultErrorCallback = new physx::PxDefaultErrorCallback;
    _foundation = PxCreateFoundation(PX_PHYSICS_VERSION, *_defaultAllocatorCallback, *_defaultErrorCallback);

    for (int i = 0; i < _coreCount; i++) {
        _coreMutexes[i] = new std::mutex;
        _cores[i] = new quadrophysx::Core(i, _coreMutexes[i], _foundation, _threadsPerCore, _scenesPerCore);
        _threads[i] = new std::thread(&Hypervisor::spinCore, this, i);
    }
}

void quadrophysx::Hypervisor::spinCore(size_t coreId) {
    while (true) {
        _cores[coreId]->spin(500000);
    }
}

size_t quadrophysx::Hypervisor::getTicks() {
    size_t s = 0;
    for (int i = 0; i < _coreCount; i++) {
        s += _cores[i]->getTicks();
    }
    return s;
}

void quadrophysx::Hypervisor::scheduleTask(quadrophysx::SimulationTask *myTask) {
    size_t queuedCore = 0;
    size_t queuedCoreSize = _cores[0]->getQueuedTasksCount();
    for (int i = 1; i < _coreCount; i++) {
        size_t tmpQueuedCoreSize = _cores[i]->getQueuedTasksCount();
        if (tmpQueuedCoreSize < queuedCoreSize) {
            queuedCore = i;
            queuedCoreSize = tmpQueuedCoreSize;
        }
    }
    _cores[queuedCore]->addTask(myTask);
}

void quadrophysx::Hypervisor::printCores() {
    for (int i = 0; i < _coreCount; i++) {
        std::cout << _cores[i]->getQueuedTasksCount() << " ";
    }
    std::cout << std::endl;
}

std::queue<quadrophysx::SimulationTask *> quadrophysx::Hypervisor::getFinishedTasksQueue() {
    std::queue<quadrophysx::SimulationTask *> queue;
    for (int i = 0; i < _coreCount; i++) {
        std::queue<quadrophysx::SimulationTask *> tmpQueue = _cores[i]->getFinishedTasksQueue();
        while (!tmpQueue.empty()) {
            queue.push(tmpQueue.front());
            tmpQueue.pop();
        }
    }
    return queue;
}

size_t quadrophysx::Hypervisor::getFinishedTasksQueueSize() {
    size_t s = 0;
    for (int i = 0; i < _coreCount; i++) {
        s += _cores[i]->getFinishedTasksQueueSize();
    }
    return s;
}

void quadrophysx::Hypervisor::clearQueue() {
    std::unique_lock<std::mutex> lock(_finishedTasksMutex);
    for (int i = 0; i < _coreCount; i++) {
        _cores[i]->clearTaskQueue();
    }
}
