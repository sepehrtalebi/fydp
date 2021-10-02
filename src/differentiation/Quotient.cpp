#include "Quotient.h"
#include "Difference.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"

double Quotient::evaluate(const std::map<std::string, double> &variables) const {
    return num->evaluate(variables) / den->evaluate(variables);
}

std::shared_ptr<Expression> Quotient::diff(const std::string &id) const {
    return (num->diff(id) * den - den->diff(id) * num) / (den * den);
}

std::shared_ptr<Expression> Quotient::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return num->subs(subs) / den->subs(subs);
}

std::string Quotient::toStr() const {
    return "(" + num->toStr() + " / " + den->toStr() + ")";
}

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2) {
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

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression> &expr, const double &num) {
    return expr / std::make_shared<Constant>(num);
}

std::shared_ptr<Expression> operator/(const double &num, const std::shared_ptr<Expression> &expr) {
    return std::make_shared<Constant>(num) / expr;
}

void operator/=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2) {
    expr1 = expr1 / expr2;
}

void operator/=(std::shared_ptr<Expression> &expr, const double &num) {
    expr = expr / num;
}

void operator/=(const double &num, std::shared_ptr<Expression> &expr) {
    expr = num / expr;
}
