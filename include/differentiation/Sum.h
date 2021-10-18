#pragma once

#include "BinaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Sum : public BinaryOperator {
public:
    Sum(ExprPtr first, ExprPtr second) : BinaryOperator(std::move(first), std::move(second)) {}

    [[nodiscard]] ExprPtr diff(const std::string &id) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

protected:
    [[nodiscard]] double call(const double &first, const double &second) const override;

    [[nodiscard]] ExprPtr call(const ExprPtr &first, const ExprPtr &second) const override;

    [[nodiscard]] std::string type() const override;

    [[nodiscard]] bool isAssociative() const override;

    [[nodiscard]] bool isCommutative() const override;
};

ExprPtr operator+(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr operator+(const ExprPtr &expr, const double &num);

ExprPtr operator+(const double &num, const ExprPtr &expr);

void operator+=(ExprPtr &expr1, const ExprPtr &expr2);

void operator+=(ExprPtr &expr, const double &num);

void operator+=(const double &num, ExprPtr &expr);
