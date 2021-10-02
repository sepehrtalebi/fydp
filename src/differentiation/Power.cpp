#include "Power.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"
#include "Sum.h"

#include <cmath>

double Power::evaluate(const std::map<std::string, double> &variables) const {
    return pow(base->evaluate(variables), power->evaluate(variables));
}

ExprPtr Power::diff(const std::string &id) const {
    return pow(base, power) * (power->diff(id) * log(base) + base->diff(id) * (power / base));
}

ExprPtr Power::subs(const std::map<std::string, ExprPtr> & subs) const {
    return std::make_shared<Power>(base->subs(subs), power->subs(subs));
}

ExprPtr Power::simplify() const {
    // try combining constants
    std::shared_ptr<Constant> base_const = std::dynamic_pointer_cast<Constant>(base);
    std::shared_ptr<Constant> power_const = std::dynamic_pointer_cast<Constant>(power);
    if (base_const && power_const) {
        return std::make_shared<Constant>(pow(base_const->getValue(), power_const->getValue()));
    }
    return pow(base->simplify(), power->simplify());
}

std::string Power::toStr() const {
    return "(" + base->toStr() + " ^ " + power->toStr() + ")";
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
