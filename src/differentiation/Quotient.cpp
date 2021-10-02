#include "Quotient.h"
#include "Difference.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"

double Quotient::evaluate(const std::map<std::string, double> &variables) const {
    return num->evaluate(variables) / den->evaluate(variables);
}

ExprPtr Quotient::diff(const std::string &id) const {
    return (num->diff(id) * den - den->diff(id) * num) / (den * den);
}

ExprPtr Quotient::subs(const std::map<std::string, ExprPtr> & subs) const {
    return num->subs(subs) / den->subs(subs);
}

ExprPtr Quotient::simplify() const {
    // try combining constants
    std::shared_ptr<Constant> num_const = std::dynamic_pointer_cast<Constant>(num);
    std::shared_ptr<Constant> den_const = std::dynamic_pointer_cast<Constant>(den);
    if (num_const && den_const) {
        return std::make_shared<Constant>(num_const->getValue() / den_const->getValue());
    }
    return num->simplify() / den->simplify();
}

std::string Quotient::toStr() const {
    return "(" + num->toStr() + " / " + den->toStr() + ")";
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
