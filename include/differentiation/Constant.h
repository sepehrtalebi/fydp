#pragma once

#include "Expression.h"

template<typename T>
class Constant : public Expression<T> {
private:
    T value;

public:
    explicit Constant(const T &value) {
        this->value = value;
    }

    T evaluate(std::map<std::string, T> /** variables **/) {
        return value;
    }

    Expression<T> diff(const std::string & /** identifier **/) {
        return Constant<T>{0};
    }
};
