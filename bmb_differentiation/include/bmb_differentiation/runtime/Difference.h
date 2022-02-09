#pragma once

#include <bmb_differentiation/runtime/BinaryOperator.h>

#include <memory>
#include <string>
#include <utility>

class Difference : public BinaryOperator {
public:
    Difference(ExprPtr first, ExprPtr second) : BinaryOperator(std::move(first), std::move(second)) {}

    [[nodiscard]] ExprPtr diff(const std::string &id) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

protected:
    [[nodiscard]] double call(const double &first, const double &second) const override;

    [[nodiscard]] ExprPtr call(const ExprPtr &first, const ExprPtr &second) const override;

    [[nodiscard]] std::string type() const override;
};
