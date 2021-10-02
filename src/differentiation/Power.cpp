#include "Power.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"

#include <cmath>

ExprPtr Power::diff(const std::string &id) const {
    return pow(first, second) * (second->diff(id) * log(first) + first->diff(id) * (second / first));
}

std::string Power::toStr() const {
    return "(" + first->toStr() + " ^ " + second->toStr() + ")";
}

double Power::call(const double &first, const double &second) const {
    return pow(first, second);
}

ExprPtr Power::call(const ExprPtr &first, const ExprPtr &second) const {
    return pow(first, second);
}

std::string Power::type() const {
    return "pow";
}

ExprPtr pow(const ExprPtr &base, const ExprPtr &power) {
    if (!base || !power) return std::make_shared<Nan>();

    if (base->isNan()) return base;
    if (power->isNan()) return power;
    if (base->isZero() && power->isZero()) return std::make_shared<One>();
    if (base->isZero() || base->isOne()) return base;
    if (power->isOne()) return base;
    if (power->isZero()) return std::make_shared<One>();
    return std::make_shared<Power>(base, power);
}

ExprPtr pow(const ExprPtr &expr, const double &num) {
    return pow(expr, std::make_shared<Constant>(num));
}

ExprPtr pow(const double &num, const ExprPtr &expr) {
    return pow(std::make_shared<Constant>(num), expr);
}

ExprPtr sqrt(const ExprPtr &expr) {
    return pow(expr, std::make_shared<Constant>(1.0 / 2.0));
}
