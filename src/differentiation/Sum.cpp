#include "Sum.h"

Sum::~Sum() {
    delete left;
    delete right;
}

double Sum::evaluate(const std::map<std::string, double> &variables) const {
    return left->evaluate(variables) + right->evaluate(variables);
}

Expression* Sum::diff(const std::string &id) const {
    return new Sum(left->diff(id), right->diff(id));
}

std::string Sum::toStr() const {
    return "(" + left->toStr() + " + " + right->toStr() + ")";
}

Expression *Sum::copy() const {
    return new Sum(left->copy(), right->copy());
}
