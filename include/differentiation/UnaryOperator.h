#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class UnaryOperator : public Expression {
public:
    UnaryOperator() = delete;

    explicit UnaryOperator(std::shared_ptr<Expression> operand) : operand(std::move(operand)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &id) const override;

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::shared_ptr<Expression> simplify() const override;

    std::string toStr() const override;

protected:
    virtual double call(const double &operand) const = 0;

    virtual std::shared_ptr<Expression> call(const std::shared_ptr<Expression> &operand) const = 0;

    virtual std::shared_ptr<Expression> derivative(const std::shared_ptr<Expression> &expr) const = 0;

    virtual std::string toStrWrapper(const std::string &operandString) const = 0;

private:
    std::shared_ptr<Expression> operand;
};

#include "Log.h"
#include "Sin.h"
#include "Cos.h"
