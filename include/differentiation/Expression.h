#pragma once

#include <string>
#include <map>

class Expression {
public:
    virtual ~Expression() = default;

    virtual double evaluate(const std::map<std::string, double> &variables) const = 0;

    virtual Expression* diff(const std::string &id) const = 0;

    virtual std::string toStr() const = 0;

    virtual Expression* copy() const = 0;
};
