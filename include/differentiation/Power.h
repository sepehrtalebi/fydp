#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Power : public Expression {
private:
    std::shared_ptr<Expression> base;
    std::shared_ptr<Expression> power;
public:
    Power(std::shared_ptr<Expression> base, std::shared_ptr<Expression> power) : base(std::move(base)), power(std::move(power)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::string toStr() const override;
};
