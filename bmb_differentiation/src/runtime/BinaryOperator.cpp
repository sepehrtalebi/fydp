#include "bmb_differentiation/runtime/BinaryOperator.h"

#include <bmb_differentiation/runtime/Constant.h>

#include <unordered_map>
#include <string>

double BinaryOperator::evaluate(const std::unordered_map<std::string, double> &variables) const {
    return call(first->evaluate(variables), second->evaluate(variables));
}

ExprPtr BinaryOperator::subs(const std::unordered_map<std::string, ExprPtr> &subs) const {
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

    std::shared_ptr<BinaryOperator> first_bin_op = std::dynamic_pointer_cast<BinaryOperator>(simplified_first);
    std::shared_ptr<BinaryOperator> second_bin_op = std::dynamic_pointer_cast<BinaryOperator>(simplified_second);

    if (isAssociative()) {
        // ignore cases with mixed operators
        bool first_bin_op_same = first_bin_op && type() == first_bin_op->type();
        bool second_bin_op_same = second_bin_op && type() == second_bin_op->type();

        // Since we're only dealing with cases where first_bin_op and second_bin_op are the same type as this,
        // they will also be commutative if this is commutative. Thus, we can ignore simplification cases that
        // rely on commutativity and contain a subexpression of the form (expr @ const) these subexpressions will be
        // converted to the form (const @ expr).

        // Handle cases of the form (const @ (const @ expr))
        if (first_const && second_bin_op && second_bin_op_same) {
            ConstPtr second_first_const = std::dynamic_pointer_cast<Constant>(second_bin_op->first);
            if (second_first_const) {
                ConstPtr combined_const = Constant::make(call(first_const->getValue(), second_first_const->getValue()));
                return call(combined_const, second_bin_op->second);
            }
        }

        // Handle cases of the form ((expr @ const) @ const)
        else if (second_const && first_bin_op && first_bin_op_same) {
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
        else if (first_bin_op && second_bin_op && first_bin_op_same && second_bin_op_same) {
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

    ExprPtr result;
    if (isCommutative() && second_const) result = call(second_const, simplified_first);
    else result = call(simplified_first, simplified_second);
    unsigned int result_node_count = 0; // lazily initialized

    // try left distributivity to see if it decreases the nodeCount
    if (second_bin_op && isDistributiveOn(second_bin_op->type())) {
        result_node_count = result->nodeCount();

        ExprPtr expr = second_bin_op->call(call(simplified_first, second_bin_op->first), call(simplified_first, second_bin_op->second))->simplify();
        unsigned int expr_node_count = expr->nodeCount();
        if (expr_node_count < result_node_count) {
            result = expr;
            result_node_count = expr_node_count;
        }
    }

    // try right distributivity to see if it decreases the nodeCount
    if (first_bin_op && isCommutative() && isDistributiveOn(first_bin_op->type())) {
        if (result_node_count == 0) result_node_count = result->nodeCount();

        ExprPtr expr = first_bin_op->call(call(first_bin_op->first, simplified_second), call(first_bin_op->second, simplified_second))->simplify();
        unsigned int expr_node_count = expr->nodeCount();
        if (expr_node_count < result_node_count) {
            result = expr;
            // No need to update expr_node_count
        }
    }

    return result;
}

unsigned int BinaryOperator::nodeCount() const {
    return 1 + first->nodeCount() + second->nodeCount();
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
