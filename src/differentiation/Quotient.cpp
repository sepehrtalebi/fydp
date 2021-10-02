#include "Quotient.h"
#include "Product.h"
#include "Difference.h"

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
