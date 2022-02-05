#ifndef QUADROPHYSX_ROBOTSTATE_H
#define QUADROPHYSX_ROBOTSTATE_H


namespace quadrophysx {

    struct JointState {
        double position;
        double velocity;
        double acceleration;
        double force;
    };

    struct LinkState {
        struct {
            physx::PxVec3 linear;
            physx::PxVec3 angular;
        } velocity, acceleration;
        physx::PxTransform transform;
    };

    struct RobotState {
        LinkState rootLink;
        LinkState links[4][3];
        JointState joints[4][3];
    };

}


#endif //QUADROPHYSX_ROBOTSTATE_H
