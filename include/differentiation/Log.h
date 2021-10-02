#pragma once

#include "UnaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Log : public UnaryOperator {
public:
    explicit Log(const std::shared_ptr<Expression> &operand) : UnaryOperator(operand) {}

protected:
    double call(const double &operand) const override;

    std::shared_ptr<Expression> call(const std::shared_ptr<Expression> &operand) const override;

    std::shared_ptr<Expression> derivative(const std::shared_ptr<Expression> &expr) const override;

    std::string toStrWrapper(const std::string &operandString) const override;
};

std::shared_ptr<Expression> log(const std::shared_ptr<Expression> &expr);
