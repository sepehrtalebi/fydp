#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Log : public Expression {
private:
    std::shared_ptr<Expression> value;
public:
    Log(const std::shared_ptr<Expression> &value) : value(value) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::string toStr() const override;
};

std::shared_ptr<Expression> log(const std::shared_ptr<Expression> &expr);
