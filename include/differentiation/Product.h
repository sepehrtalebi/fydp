#pragma once

#include "BinaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Product : public BinaryOperator {
public:
    Product(ExprPtr first, ExprPtr second) : BinaryOperator(std::move(first), std::move(second)) {}

    [[nodiscard]] ExprPtr diff(const std::string &id) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

protected:
    [[nodiscard]] double call(const double &first, const double &second) const override;

    [[nodiscard]] ExprPtr call(const ExprPtr &first, const ExprPtr &second) const override;

    [[nodiscard]] std::string type() const override;

    [[nodiscard]] bool isAssociative() const override;

    [[nodiscard]] bool isCommutative() const override;

    [[nodiscard]] bool isDistributiveOn(const std::string &type) const override;
};
