#include "Difference.h"
#include "Zero.h"
#include "Constant.h"
#include "Variable.h"

double Difference::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) - right->evaluate(variables);
}

std::shared_ptr<Expression> Difference::diff(const std::string &id) const {
    return left->diff(id) - right->diff(id);
}

std::shared_ptr<Expression> Difference::subs(const std::map <std::string, std::shared_ptr<Expression>> &subs) const {
    return left->subs(subs) - right->subs(subs);
}

std::shared_ptr<Expression> Difference::simplify() const {
    // try combining variables with the same identifier
    std::shared_ptr<Variable> left_var = std::dynamic_pointer_cast<Variable>(left);
    std::shared_ptr<Variable> right_var = std::dynamic_pointer_cast<Variable>(right);
    if (left_var && right_var && left_var->getIdentifier() == right_var->getIdentifier()) {
        return std::make_shared<Zero>();
    }

    // try combining constants
    std::shared_ptr<Constant> left_const = std::dynamic_pointer_cast<Constant>(left);
    std::shared_ptr<Constant> right_const = std::dynamic_pointer_cast<Constant>(right);
    if (left_const && right_const) {
        return std::make_shared<Constant>(left_const->getValue() - right_const->getValue());
    }

    return left->simplify() - right->simplify();
}

std::string Difference::toStr() const {
    return "(" + left->toStr() + " - " + right->toStr() + ")";
}

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2) {
    if (!expr1 && !expr2) return std::make_shared<Zero>();
    if (!expr1) return std::make_shared<Product>(std::make_shared<Constant>(-1), expr2);
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr2->isZero()) return expr1;
    if (expr1->isZero()) return std::make_shared<Product>(std::make_shared<Constant>(-1), expr2);
    return std::make_shared<Difference>(expr1, expr2);
}

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression> &expr) {
    if (!expr) return std::make_shared<Zero>();

    if (expr->isNan() || expr->isZero()) return expr;
    return std::make_shared<Product>(std::make_shared<Constant>(-1), expr);
}

std::shared_ptr<Expression> operator-(const double &num, const std::shared_ptr<Expression> &expr) {
    return std::make_shared<Constant>(num) - expr;
}

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression> &expr, const double &num) {
    return expr - std::make_shared<Constant>(num);
}

void operator-=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2) {
    expr1 = expr1 - expr2;
}

void operator-=(std::shared_ptr<Expression> &expr, const double &num) {
    expr = expr - num;
}

void operator-=(const double &num, std::shared_ptr<Expression> &expr) {
    expr = num - expr;
}