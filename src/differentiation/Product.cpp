#include "Product.h"
#include "Sum.h"

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
