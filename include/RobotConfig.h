#ifndef QUADROPHYSX_ROBOTCONFIG_H
#define QUADROPHYSX_ROBOTCONFIG_H


namespace quadrophysx {

    struct JointConfig {
        float limitLower;
        float limitUpper;
        float position;
        float stiffness = 10e6;
        float damping = 0;
        float maxForce = PX_MAX_F32;
    };

    struct LinkConfig {
        physx::PxVec3 size;
        float density;
    };

    struct RobotConfig {
        LinkConfig rootLink;
        LinkConfig links[4][3];
        JointConfig joints[4][3];
    };

}


#endif //QUADROPHYSX_ROBOTCONFIG_H
