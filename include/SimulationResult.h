#ifndef QUADROPHYSX_SIMULATIONRESULT_H
#define QUADROPHYSX_SIMULATIONRESULT_H


#include "Strategy.h"

namespace quadrophysx {

    class SimulationResult {
    private:
        int _id;
        quadrophysx::Strategy *_strategy;
    public:
        SimulationResult(int id, quadrophysx::Strategy *servoConfig) : _id(id), _strategy(servoConfig) {};

        int getId() { return _id; };

        quadrophysx::Strategy *getStrategy() { return _strategy; }
    };

}




#endif //QUADROPHYSX_SIMULATIONRESULT_H
