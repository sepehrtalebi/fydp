#pragma once

#include <string>
#include <map>

template<typename T>
class Expression {
public:
    virtual T evaluate(std::map<std::string, T> variables) = 0;

    virtual Expression<T> diff(const std::string &identifier) = 0;
};
