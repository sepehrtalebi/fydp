#include "Difference.h"

double Difference::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) - right->evaluate(variables);
}

std::shared_ptr<Expression> Difference::diff(const std::string &id) const {
    return left->diff(id) - right->diff(id);
}

std::shared_ptr<Expression> Difference::subs(const std::map <std::string, std::shared_ptr<Expression>> &subs) const {
    return left->subs(subs) - right->subs(subs);
}

std::string Difference::toStr() const {
    return "(" + left->toStr() + " - " + right->toStr() + ")";
}
