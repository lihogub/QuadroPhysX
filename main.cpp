#include <iostream>
#include "PxPhysicsAPI.h"
#include "RobotConfig.h"
#include "Hypervisor.h"
#include "task/SimulationTask.h"
#include <map>
#include <algorithm>

quadrophysx::RobotConfig getRobotConfig() {
    quadrophysx::RobotConfig robotConfig{};

    physx::PxVec3 sizeAlpha(5, 5, 10);
    physx::PxVec3 sizeBeta(5, 20, 5);
    physx::PxVec3 sizeGamma(5, 20, 5);

    robotConfig.rootLink.size = physx::PxVec3(40, 10, 20);
    robotConfig.rootLink.density = 80;

    for (int i = 0; i < 4; i++) {
        robotConfig.links[i][0].size = sizeAlpha;
        robotConfig.links[i][0].density = 10;
        robotConfig.links[i][1].size = sizeBeta;
        robotConfig.links[i][1].density = 10;
        robotConfig.links[i][2].size = sizeGamma;
        robotConfig.links[i][2].density = 10;

        robotConfig.joints[i][0].position = 0;
        robotConfig.joints[i][0].limitLower = -physx::PxPi / 2;
        robotConfig.joints[i][0].limitUpper = physx::PxPi / 2;
        robotConfig.joints[i][0].stiffness = 10e10;
        robotConfig.joints[i][0].maxForce = 5 * 10e9;

        robotConfig.joints[i][1].position = -physx::PxPi / 4;
        robotConfig.joints[i][1].limitLower = -physx::PxPi / 2;
        robotConfig.joints[i][1].limitUpper = physx::PxPi / 2;
        robotConfig.joints[i][1].stiffness = 10e10;
        robotConfig.joints[i][1].maxForce = 5 * 10e9;

        robotConfig.joints[i][2].position = physx::PxPi / 2;
        robotConfig.joints[i][2].limitLower = -physx::PxPi / 2;
        robotConfig.joints[i][2].limitUpper = physx::PxPi / 2;
        robotConfig.joints[i][2].stiffness = 10e10;
        robotConfig.joints[i][2].maxForce = 5 * 10e9;
    }
    return robotConfig;
}

void init(quadrophysx::Strategy *servoConfig, size_t count) {
    for (int k = 0; k < count; k++) {
        for (int leg = 0; leg < 4; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                servoConfig[k].value[leg][joint] = ((rand() % 101) - 50) / 1000.0;
            }
        }
    }


}

bool comp(quadrophysx::SimulationTask *task1, quadrophysx::SimulationTask *task2) {
    return task1->getResult() > task2->getResult();
}

void mutate(quadrophysx::Strategy *strategy, size_t frequencies) {
    for (int i = 0; i < frequencies; i++) {
        for (int leg = 0; leg < 4; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                strategy[i].value[leg][joint] += ((rand() % 101) - 50) / 200.0;
            }
        }
    }
}

int main() {
    quadrophysx::RobotConfig robotConfig = getRobotConfig();

    quadrophysx::Hypervisor *hypervisor = new quadrophysx::Hypervisor(4, 4, 64);
    hypervisor->addConfiguration(robotConfig);
    hypervisor->run();

    srand(time(nullptr));
    size_t taskCount = 1000;
    size_t epochs = 30000;
    size_t frequencies = 3;
    auto *map = new std::map<int, quadrophysx::Strategy *>();

    for (int i = 0; i < taskCount; i++) {
        quadrophysx::Strategy *servoConfig = new quadrophysx::Strategy[frequencies];
        init(servoConfig, frequencies);
        map->insert_or_assign(i, servoConfig);
    }

    for (int i = 0; i < taskCount; i++) {
        std::cout << i << std::endl;
        quadrophysx::Strategy *servoConfig = map->at(i);
        auto *task = new quadrophysx::SimulationTask(i, epochs, 1.0f / 100.0f, servoConfig, &robotConfig);
        hypervisor->submitTask(task);
    }

    size_t last = 0;
    size_t deltaTime = 5000;

    while (true) {
        hypervisor->spin(deltaTime);
        size_t cur = hypervisor->getTicks();

        std::cout
                << "ticks=" << cur << " "
                << "speed=" << 1.0 * (cur - last) / deltaTime * 1000 << " "
                << "finishedTasksQueueSize=" << hypervisor->getFinishedTasksQueueSize() << std::endl;
        last = cur;


        int edenCount = 100;
//        hypervisor->printCores();
        if (hypervisor->getFinishedTasksQueueSize() == taskCount) {

            auto vec = hypervisor->fetchTasks();
            hypervisor->clearQueue();
            std::sort(vec.begin(), vec.end(), comp);
            double s = 0.0;
            double maxId = vec[0]->getId();
            double maxVal = vec[0]->getResult();
            for (int i = 0; i < edenCount; i++) {
                auto task = vec[i];
//                std::cout << task->getId() << " " << task->getResult() << std::endl;
                s += task->getResult();
                auto *task1 = new quadrophysx::SimulationTask(task->getId(), epochs, 1.0f / 100.0f, map->at(vec[i]->getId()), &robotConfig);
                hypervisor->submitTask(task1);
            }

            std::cout << "mean: " << s / edenCount << " maxId: " << maxId << " maxVal: " << maxVal << std::endl;
            for (int leg = 0; leg < 4; leg++) {
                for (int joint = 0; joint < 3; joint++) {
                    std::cout << leg << " " << joint << " ";
                    for (int f = 0; f < frequencies; f++) {
                        std::cout << vec[maxId]->getStrategy()[f].value[leg][joint] << " ";
                    }
                    std::cout << std::endl;
                }
            }

            std::cout << std::endl;

            for (int j = edenCount; j < taskCount; j++) {
                memcpy_s(map->at(vec[j]->getId()), frequencies, map->at(vec[j % edenCount]->getId()), frequencies);
                mutate(map->at(vec[j]->getId()), frequencies);
                auto *task = new quadrophysx::SimulationTask(vec[j]->getId(), epochs, 1.0f / 100.0f, map->at(vec[j]->getId()), &robotConfig);
                hypervisor->submitTask(task);
            }

        }
    }

    return 0;
}