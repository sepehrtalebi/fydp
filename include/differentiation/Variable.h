#pragma once

#include <utility>
#include "Expression.h"
#include "Constant.h"

class Variable : public Expression {
private:
    std::string identifier;
public:
    explicit Variable(std::string id) : identifier(std::move(id)) {}

    ~Variable() override = default;

    double evaluate(const std::map<std::string, double> &variables) const override;

    Expression* diff(const std::string &id) const override;

    std::string toStr() const override;

    Expression* copy() const override;
};
