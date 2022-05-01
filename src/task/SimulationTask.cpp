#include <iostream>
#include "task/SimulationTask.h"

quadrophysx::SimulationTask::SimulationTask(int id, size_t epochs, float elapsedTime, quadrophysx::Strategy *strategy,
                                            quadrophysx::RobotConfig *robotConfig)
        : _id(id), _epochs(epochs), _elapsedTime(elapsedTime), _strategy(strategy), _robotConfig(robotConfig) {
}

void quadrophysx::SimulationTask::setup(physx::PxPhysics *physics, physx::PxSceneDesc *sceneDesc) {
    _physics = physics;
    _sceneDesc = sceneDesc;
}

quadrophysx::SimulationTask::Status quadrophysx::SimulationTask::onStatusCheck() {
    if (_epoch == _epochs) {
        _status = FINISHED;
    }
    return _status;
}

size_t quadrophysx::SimulationTask::onFirstRun() {
    _scene = _physics->createScene(*_sceneDesc);
    physx::PxMaterial *_material = _physics->createMaterial(0.5f, 0.5f, 0.5f);
    physx::PxRigidStatic* _groundPlane = PxCreatePlane(*_physics, physx::PxPlane(0, 1, 0, 0), *_material);
    _scene->addActor(*_groundPlane);


    _robot = new Robot(_physics, _scene, *_robotConfig);

    _robot->createCache();
    _robot->updateRobotState();

    _scene->simulate(_elapsedTime);
    ++_epoch;

    _noise = 0.0;

    _status = RUNNING;

    return 1;
}

size_t quadrophysx::SimulationTask::onBeforeRun() {

    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {

            double val = 0.0;
            double x = 2.0 * _epoch / _epochs * physx::PxTwoPi;
            for (int n = 1; n <= 3; n++) {
                val += sin(x * n) * _strategy[n - 1].value[leg][joint];
            }

            val = (val > 1) ? 1 : val;
            val = (val < -1) ? -1 : val;

            val = (val + 1) / 2.0;
            _robot->setTargetPosition(leg, joint, val);
        }
    }

    return 0;
}

size_t quadrophysx::SimulationTask::onRun() {

    if (_scene->fetchResults(false)) {


        _robot->updateRobotState();

        double q = 0;
        q += pow(_robot->getRobotState()->rootLink.transform.q.x, 2);
        q += pow(_robot->getRobotState()->rootLink.transform.q.y, 2);
        q += pow(_robot->getRobotState()->rootLink.transform.q.z, 2);
        q += pow(_robot->getRobotState()->rootLink.transform.q.w - 1, 2);

        _noise += sqrt(q);

        _scene->simulate(_elapsedTime);
        ++_epoch;

        return 1;
    }

    return 0;
}

size_t quadrophysx::SimulationTask::onAfterRun() {
    return 0;
}

size_t quadrophysx::SimulationTask::onFinish() {
    _scene->fetchResults(true);

    _scene->release();

    _robot->updateRobotState();

    return 0;

}

double quadrophysx::SimulationTask::getResult() const {
    double s = 0;
    s += _robot->getRobotState()->rootLink.transform.p.x;
    s -= pow(_robot->getRobotState()->rootLink.transform.p.y - 49.5, 2);
    s -= pow(_robot->getRobotState()->rootLink.transform.p.z, 2);

    return s / pow(1 + _noise, 2);
}

quadrophysx::Strategy *quadrophysx::SimulationTask::getStrategy() const {
    return _strategy;
}
