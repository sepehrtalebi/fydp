#include "Sum.h"
#include "Zero.h"
#include "Constant.h"
#include "Variable.h"

double Sum::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) + right->evaluate(variables);
}

ExprPtr Sum::diff(const std::string &id) const {
    return left->diff(id) + right->diff(id);
}

ExprPtr Sum::subs(const std::map <std::string, ExprPtr> &subs) const {
    return left->subs(subs) + right->subs(subs);
}

ExprPtr Sum::simplify() const {
    // try combining variables with the same identifier
    std::shared_ptr<Variable> left_var = std::dynamic_pointer_cast<Variable>(left);
    std::shared_ptr<Variable> right_var = std::dynamic_pointer_cast<Variable>(right);
    if (left_var && right_var && left_var->getIdentifier() == right_var->getIdentifier()) {
        return 2 * left_var;
    }

    // try combining constants
    std::shared_ptr<Constant> left_const = std::dynamic_pointer_cast<Constant>(left);
    std::shared_ptr<Constant> right_const = std::dynamic_pointer_cast<Constant>(right);
    if (left_const && right_const) {
        return std::make_shared<Constant>(left_const->getValue() + right_const->getValue());
    }

    return left->simplify() + right->simplify();
}

std::string Sum::toStr() const {
    return "(" + left->toStr() + " + " + right->toStr() + ")";
}

ExprPtr operator+(const ExprPtr &expr1, const ExprPtr &expr2) {
    if (!expr1 && !expr2) return std::make_shared<Zero>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr1->isZero()) return expr2;
    if (expr2->isZero()) return expr1;
    return std::make_shared<Sum>(expr1, expr2);
}

ExprPtr operator+(const ExprPtr &expr, const double &num) {
    return expr + std::make_shared<Constant>(num);
}

ExprPtr operator+(const double &num, const ExprPtr &expr) {
    return expr + std::make_shared<Constant>(num);
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