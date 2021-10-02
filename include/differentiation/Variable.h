#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Variable : public Expression {
private:
    std::string identifier;
public:
    explicit Variable(std::string id) : identifier(std::move(id)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::string toStr() const override;
};
