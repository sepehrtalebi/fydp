#include "Sum.h"

double Sum::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) + right->evaluate(variables);
}

std::shared_ptr<Expression> Sum::diff(const std::string &id) const {
    return left->diff(id) +  right->diff(id);
}

std::string Sum::toStr() const {
    return "(" + left->toStr() + " + " + right->toStr() + ")";
}
