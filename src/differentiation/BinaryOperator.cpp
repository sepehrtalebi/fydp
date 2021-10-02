#include "BinaryOperator.h"
#include "Constant.h"

double BinaryOperator::evaluate(const std::map<std::string, double> &variables) const {
    return call(first->evaluate(variables), second->evaluate(variables));
}

ExprPtr BinaryOperator::subs(const std::map<std::string, ExprPtr> &subs) const {
    return call(first->subs(subs), second->subs(subs));
}

ExprPtr BinaryOperator::simplify() const {
    ExprPtr simplified_first = first->simplify();
    ExprPtr simplified_second = second->simplify();

    // try combining constants
    ConstPtr first_const = std::dynamic_pointer_cast<Constant>(simplified_first);
    ConstPtr second_const = std::dynamic_pointer_cast<Constant>(simplified_second);
    if (first_const && second_const) {
        return Constant::make(call(first_const->getValue(), second_const->getValue()));
    }

    if (isAssociative()) {
        std::shared_ptr<BinaryOperator> first_bin_op = std::dynamic_pointer_cast<BinaryOperator>(simplified_first);
        std::shared_ptr<BinaryOperator> second_bin_op = std::dynamic_pointer_cast<BinaryOperator>(simplified_second);

        // ignore cases with mixed operators
        if (first_bin_op && type() != first_bin_op->type()) first_bin_op = nullptr;
        if (second_bin_op && type() != second_bin_op->type()) second_bin_op = nullptr;

        // Since we're only dealing with cases where first_bin_op and second_bin_op are the same type as this,
        // they will also be commutative if this is commutative. Thus, we can ignore simplification cases that
        // rely on commutativity and contain a subexpression of the form (expr @ const) these subexpressions will be
        // converted to the form (const @ expr).

        // Handle cases of the form (const @ (const @ expr))
        if (first_const && second_bin_op) {
            ConstPtr second_first_const = std::dynamic_pointer_cast<Constant>(second_bin_op->first);
            if (second_first_const) {
                ConstPtr combined_const = Constant::make(call(first_const->getValue(), second_first_const->getValue()));
                return call(combined_const, second_bin_op->second);
            }
        }

        // Handle cases of the form ((expr @ const) @ const)
        else if (second_const && first_bin_op) {
            ConstPtr first_second_const = std::dynamic_pointer_cast<Constant>(first_bin_op->second);
            if (first_second_const) {
                ConstPtr combined_const = Constant::make(call(first_second_const->getValue(), second_const->getValue()));
                return call(first_bin_op->first, combined_const);
            }

            if (isCommutative()) {
                // Handle cases of the form ((const @ expr) @ const)
                ConstPtr first_first_const = std::dynamic_pointer_cast<Constant>(first_bin_op->first);
                if (first_first_const) {
                    ConstPtr combined_const = Constant::make(call(first_first_const->getValue(), second_const->getValue()));
                    return call(combined_const, first_bin_op->second);
                }
            }
        }

        // Handle cases of the form ((expr @ const) @ (const @ expr))
        else if (first_bin_op && second_bin_op) {
            ConstPtr first_second_const = std::dynamic_pointer_cast<Constant>(first_bin_op->second);
            ConstPtr second_first_const = std::dynamic_pointer_cast<Constant>(second_bin_op->first);
            if (first_second_const && second_first_const) {
                ConstPtr combined_const = Constant::make(
                        call(first_second_const->getValue(), second_first_const->getValue()));
                // bracket choice is arbitrary
                return call(first_bin_op->first, call(combined_const, second_bin_op->second));
            }

            if (isCommutative()) {
                // Handle cases of the form ((const @ expr) @ (const @ expr))
                ConstPtr first_first_const = std::dynamic_pointer_cast<Constant>(first_bin_op->first);
                if (first_first_const && second_first_const) {
                    ConstPtr combined_const = Constant::make(
                            call(first_first_const->getValue(), second_first_const->getValue()));
                    return call(combined_const, call(first_bin_op->second, second_bin_op->second));
                }
            }
        }
    }

    if (isCommutative() && second_const) return call(second_const, simplified_first);
    return call(simplified_first, simplified_second);
}

bool BinaryOperator::isAssociative() const {
    return false;
}

bool BinaryOperator::isCommutative() const {
    return false;
}

bool BinaryOperator::isDistributiveOn(const std::string &  /** type **/) const {
    return false;
}