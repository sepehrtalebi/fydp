#pragma once

#include "Expression.h"
#include "Constant.h"

template<typename T>
class Variable : public Expression<T> {
private:
    std::string id;
public:
    explicit Variable(const std::string &identifier) {
        this->id = identifier;
    }

    T evaluate(std::map<std::string, T> variables) {
        return variables[id];
    }

    Expression<T> diff(const std::string &identifier) {
        return Constant<T>{this->id == identifier ? 1 : 0};
    }
};
