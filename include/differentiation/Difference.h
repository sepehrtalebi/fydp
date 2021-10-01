#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Difference : public Expression {
private:
    std::shared_ptr<Expression> left;
    std::shared_ptr<Expression> right;
public:
    Difference(std::shared_ptr<Expression> left, std::shared_ptr<Expression> right) : left(std::move(left)), right(std::move(right)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::string toStr() const override;
};
