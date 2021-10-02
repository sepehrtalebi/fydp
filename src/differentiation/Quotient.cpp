#include "Quotient.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"
#include "Variable.h"

ExprPtr Quotient::diff(const std::string &id) const {
    return (first->diff(id) * second - second->diff(id) * first) / (second * second);
}

ExprPtr Quotient::simplify() const {
    std::shared_ptr<Variable> first_var = std::dynamic_pointer_cast<Variable>(first);
    std::shared_ptr<Variable> second_var = std::dynamic_pointer_cast<Variable>(second);
    if (first_var && second_var && first_var->getIdentifier() == second_var->getIdentifier()) {
        return std::make_shared<One>();
    }

    return BinaryOperator::simplify();
}

std::string Quotient::toStr() const {
    return "(" + first->toStr() + " / " + second->toStr() + ")";
}

double Quotient::call(const double &first, const double &second) const {
    return first / second;
}

ExprPtr Quotient::call(const ExprPtr &first, const ExprPtr &second) const {
    return first / second;
}

std::string Quotient::type() const {
    return "/";
}

ExprPtr operator/(const ExprPtr &expr1, const ExprPtr &expr2) {
    if (!expr1 && !expr2) return std::make_shared<One>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr2->isZero()) return std::make_shared<Nan>();
    if (expr2->isOne()) return expr1;
    if (expr1->isZero()) return expr1;
    // no special case needed if expr1->isOne()
    return std::make_shared<Quotient>(expr1, expr2);
}

ExprPtr operator/(const ExprPtr &expr, const double &num) {
    return expr / std::make_shared<Constant>(num);
}

ExprPtr operator/(const double &num, const ExprPtr &expr) {
    return std::make_shared<Constant>(num) / expr;
}

void operator/=(ExprPtr &expr1, const ExprPtr &expr2) {
    expr1 = expr1 / expr2;
}

void operator/=(ExprPtr &expr, const double &num) {
    expr = expr / num;
}

void operator/=(const double &num, ExprPtr &expr) {
    expr = num / expr;
}
