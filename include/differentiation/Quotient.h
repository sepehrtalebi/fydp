#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Quotient : public Expression {
private:
    std::shared_ptr<Expression> num;
    std::shared_ptr<Expression> den;
public:
    Quotient(std::shared_ptr<Expression> num, std::shared_ptr<Expression> den) : num(std::move(num)), den(std::move(den)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::string toStr() const override;
};
