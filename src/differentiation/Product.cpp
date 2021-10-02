#include "Product.h"
#include "Sum.h"
#include "One.h"
#include "Constant.h"

double Product::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) * right->evaluate(variables);
}

std::shared_ptr<Expression> Product::diff(const std::string &id) const {
    return left->diff(id) * right + right->diff(id) * left;
}

std::shared_ptr<Expression> Product::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return left->subs(subs) * right->subs(subs);
}

std::string Product::toStr() const {
    return "(" + left->toStr() + " * " + right->toStr() + ")";
}

std::shared_ptr <Expression> operator*(const std::shared_ptr <Expression> &expr1, const std::shared_ptr <Expression> &expr2) {
    if (!expr1 && !expr2) return std::make_shared<One>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr1->isZero()) return expr1;
    if (expr2->isZero()) return expr2;
    if (expr1->isOne()) return expr2;
    if (expr2->isOne()) return expr1;
    return std::make_shared<Product>(expr1, expr2);
}

std::shared_ptr <Expression> operator*(const std::shared_ptr <Expression> &expr, const double &num) {
    return expr * std::make_shared<Constant>(num);
}

std::shared_ptr <Expression> operator*(const double &num, const std::shared_ptr <Expression> &expr) {
    return std::make_shared<Constant>(num) * expr;
}

void operator*=(std::shared_ptr <Expression> &expr1, const std::shared_ptr <Expression> &expr2) {
    expr1 = expr1 * expr2;
}

void operator*=(std::shared_ptr <Expression> &expr, const double &num) {
    expr = expr * num;
}

void operator*=(const double &num, std::shared_ptr <Expression> &expr) {
    expr = num * expr;
}
