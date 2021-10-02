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

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::string toStr() const override;
};

std::shared_ptr<Expression> pow(const std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

std::shared_ptr<Expression> pow(const std::shared_ptr<Expression> &expr, double &num);

std::shared_ptr<Expression> pow(const double &num, const std::shared_ptr<Expression> &expr);

std::shared_ptr<Expression> sqrt(const std::shared_ptr<Expression> &expr);
