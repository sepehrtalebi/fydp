#include "Difference.h"
#include "Zero.h"
#include "Constant.h"
#include "Variable.h"

ExprPtr Difference::diff(const std::string &id) const {
    return first->diff(id) - second->diff(id);
}

ExprPtr Difference::simplify() const {
    std::shared_ptr<Variable> first_var = std::dynamic_pointer_cast<Variable>(first);
    std::shared_ptr<Variable> second_var = std::dynamic_pointer_cast<Variable>(second);
    if (first_var && second_var && first_var->getIdentifier() == second_var->getIdentifier()) {
        return std::make_shared<Zero>();
    }

    return BinaryOperator::simplify();
}

std::string Difference::toStr() const {
    return "(" + first->toStr() + " - " + second->toStr() + ")";
}

double Difference::call(const double &first, const double &second) const {
    return first - second;
}

ExprPtr Difference::call(const ExprPtr &first, const ExprPtr &second) const {
    return first - second;
}

std::string Difference::type() const {
    return "-";
}

ExprPtr operator-(const ExprPtr &expr1, const ExprPtr &expr2) {
    if (!expr1 && !expr2) return std::make_shared<Zero>();
    if (!expr1) return std::make_shared<Product>(std::make_shared<Constant>(-1), expr2);
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr2->isZero()) return expr1;
    if (expr1->isZero()) return std::make_shared<Product>(std::make_shared<Constant>(-1), expr2);
    return std::make_shared<Difference>(expr1, expr2);
}

ExprPtr operator-(const ExprPtr &expr) {
    if (!expr) return std::make_shared<Zero>();

    if (expr->isNan() || expr->isZero()) return expr;
    return std::make_shared<Product>(std::make_shared<Constant>(-1), expr);
}

ExprPtr operator-(const double &num, const ExprPtr &expr) {
    return std::make_shared<Constant>(num) - expr;
}

ExprPtr operator-(const ExprPtr &expr, const double &num) {
    return expr - std::make_shared<Constant>(num);
}

void operator-=(ExprPtr &expr1, const ExprPtr &expr2) {
    expr1 = expr1 - expr2;
}

void operator-=(ExprPtr &expr, const double &num) {
    expr = expr - num;
}

void operator-=(const double &num, ExprPtr &expr) {
    expr = num - expr;
}