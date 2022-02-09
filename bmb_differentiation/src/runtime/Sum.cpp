#include "bmb_differentiation/runtime/Sum.h"

#include <bmb_differentiation/runtime/Zero.h>
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/Constant.h>
#include <bmb_differentiation/runtime/Variable.h>

#include <string>
#include <memory>

ExprPtr Sum::diff(const std::string &id) const {
    return first->diff(id) + second->diff(id);
}

ExprPtr Sum::simplify() const {
    std::shared_ptr<Variable> first_var = std::dynamic_pointer_cast<Variable>(first);
    std::shared_ptr<Variable> second_var = std::dynamic_pointer_cast<Variable>(second);
    if (first_var && second_var && first_var->getIdentifier() == second_var->getIdentifier()) {
        return 2 * first_var;
    }

    return BinaryOperator::simplify();
}

std::string Sum::toStr() const {
    return "(" + first->toStr() + " + " + second->toStr() + ")";
}

double Sum::call(const double &first, const double &second) const {
    return first + second;
}

ExprPtr Sum::call(const ExprPtr &first, const ExprPtr &second) const {
    return first + second;
}

std::string Sum::type() const {
    return "+";
}

bool Sum::isAssociative() const {
    return true;
}

bool Sum::isCommutative() const {
    return true;
}

ExprPtr operator+(const ExprPtr &expr1, const ExprPtr &expr2) {
    if (!expr1 && !expr2) return Zero::INSTANCE;
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1 == Nan::INSTANCE) return expr1;
    if (expr2 == Nan::INSTANCE) return expr2;
    if (expr1 == Zero::INSTANCE) return expr2;
    if (expr2 == Zero::INSTANCE) return expr1;
    return std::make_shared<Sum>(expr1, expr2);
}

ExprPtr operator+(const ExprPtr &expr, const double &num) {
    return expr + Constant::make(num);
}

ExprPtr operator+(const double &num, const ExprPtr &expr) {
    return expr + Constant::make(num);
}

void operator+=(ExprPtr &expr1, const ExprPtr &expr2) {
    expr1 = expr1 + expr2;
}

void operator+=(ExprPtr &expr, const double &num) {
    expr = expr + num;
}

void operator+=(const double &num, ExprPtr &expr) {
    expr = expr + num;
}