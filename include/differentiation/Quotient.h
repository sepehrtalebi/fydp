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

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::string toStr() const override;
};

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression> &expr, const double &num);

std::shared_ptr<Expression> operator/(const double &num, const std::shared_ptr<Expression> &expr);

void operator/=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

void operator/=(std::shared_ptr<Expression> &expr, const double &num);

void operator/=(const double &num, std::shared_ptr<Expression> &expr);