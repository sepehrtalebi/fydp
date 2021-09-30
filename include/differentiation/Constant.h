#pragma once

#include "Expression.h"

class Constant : public Expression {
private:
    double value;

public:
    explicit Constant(const double &value) : value(value) {}

    ~Constant() override = default;

    double evaluate(const std::map<std::string, double> &variables) const override;

    Expression* diff(const std::string &identifier) const override;

    std::string toStr() const override;

    Expression* copy() const override;
};
