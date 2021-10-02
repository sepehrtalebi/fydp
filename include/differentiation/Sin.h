#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Sin : public Expression {
private:
    std::shared_ptr<Expression> value;
public:
    Sin(std::shared_ptr<Expression> value) : value(std::move(value)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::shared_ptr<Expression> simplify() const override;

    std::string toStr() const override;
};

std::shared_ptr<Expression> sin(const std::shared_ptr<Expression> &expr);
