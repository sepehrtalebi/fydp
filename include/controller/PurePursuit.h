#pragma once

#include "DubinsPath.h"

template<typename T>
class PurePursuit {
public:
    struct Result {
        bool should_replan;
        T angular_vel;
    };

    /**
     *
     * @param path
     * @param state
     * @return Commanded angular velocity
     */
    Result pursue(const DubinsPath<T>& path, const typename DubinsPath<T>::State& state) {

    }
};