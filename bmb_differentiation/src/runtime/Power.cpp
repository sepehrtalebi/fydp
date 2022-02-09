#include "bmb_differentiation/runtime/Power.h"

#include <bmb_differentiation/runtime/Zero.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/Constant.h>

#include <cmath>
#include <string>
#include <memory>

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
    if (!base || !power) return Nan::INSTANCE;

    if (base == Nan::INSTANCE) return base;
    if (power == Nan::INSTANCE) return power;
    if (base == Zero::INSTANCE && power == Zero::INSTANCE) return One::INSTANCE;
    if (base == Zero::INSTANCE || base == One::INSTANCE) return base;
    if (power == One::INSTANCE) return base;
    if (power == Zero::INSTANCE) return One::INSTANCE;
    return std::make_shared<Power>(base, power);
}

ExprPtr pow(const ExprPtr &expr, const double &num) {
    return pow(expr, Constant::make(num));
}

ExprPtr pow(const double &num, const ExprPtr &expr) {
    return pow(Constant::make(num), expr);
}

ExprPtr sqrt(const ExprPtr &expr) {
    return pow(expr, Constant::make(1. / 2.));
}
