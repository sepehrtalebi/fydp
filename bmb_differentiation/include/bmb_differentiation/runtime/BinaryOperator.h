#pragma once

#include <bmb_differentiation/runtime/Expression.h>

#include <unordered_map>
#include <memory>
#include <string>
#include <utility>

class BinaryOperator : public Expression {
public:
    BinaryOperator() = delete;

    explicit BinaryOperator(ExprPtr first, ExprPtr second) : first(std::move(first)), second(std::move(second)) {}

    [[nodiscard]] double evaluate(const std::unordered_map<std::string, double> &variables) const final;

    [[nodiscard]] ExprPtr subs(const std::unordered_map<std::string, ExprPtr> &subs) const final;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] unsigned int nodeCount() const final;

protected:
    ExprPtr first;
    ExprPtr second;

    [[nodiscard]] virtual double call(const double &first, const double &second) const = 0;

    [[nodiscard]] virtual ExprPtr call(const ExprPtr &first, const ExprPtr &second) const = 0;

    [[nodiscard]] virtual std::string type() const = 0;

    [[nodiscard]] virtual bool isAssociative() const;

    [[nodiscard]] virtual bool isCommutative() const;

    // This function only guarantees the left distributive property. The right distributive property is implied given
    // distributivity and commutativity.
    [[nodiscard]] virtual bool isDistributiveOn(const std::string &type) const;
};
