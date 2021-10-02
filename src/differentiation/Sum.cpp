#include "Sum.h"
#include "Zero.h"
#include "Constant.h"

double Sum::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) + right->evaluate(variables);
}

std::shared_ptr<Expression> Sum::diff(const std::string &id) const {
    return left->diff(id) + right->diff(id);
}

std::shared_ptr<Expression> Sum::subs(const std::map <std::string, std::shared_ptr<Expression>> &subs) const {
    return left->subs(subs) + right->subs(subs);
}

std::string Sum::toStr() const {
    return "(" + left->toStr() + " + " + right->toStr() + ")";
}

std::shared_ptr<Expression> operator+(const std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2) {
    if (!expr1 && !expr2) return std::make_shared<Zero>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr1->isZero()) return expr2;
    if (expr2->isZero()) return expr1;
    return std::make_shared<Sum>(expr1, expr2);
}

std::shared_ptr<Expression> operator+(const std::shared_ptr<Expression> &expr, const double &num) {
    return expr + std::make_shared<Constant>(num);
}

std::shared_ptr<Expression> operator+(const double &num, const std::shared_ptr<Expression> &expr) {
    return std::make_shared<Constant>(num) + expr;
}

void operator+=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2) {
    expr1 = expr1 + expr2;
}

void operator+=(std::shared_ptr<Expression> &expr, const double &num) {
    expr = expr + num;
}


void operator+=(const double &num, std::shared_ptr<Expression> &expr) {
    expr = num + expr;
}