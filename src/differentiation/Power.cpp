#include "Power.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"
#include "Sum.h"

#include <cmath>

double Power::evaluate(const std::map<std::string, double> &variables) const {
    return pow(base->evaluate(variables), power->evaluate(variables));
}

std::shared_ptr<Expression> Power::diff(const std::string &id) const {
    return pow(base, power) * (power->diff(id) * log(base) + base->diff(id) * (power / base));
}

std::shared_ptr<Expression> Power::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return std::make_shared<Power>(base->subs(subs), power->subs(subs));
}

std::string Power::toStr() const {
    return "(" + base->toStr() + " ^ " + power->toStr() + ")";
}

std::shared_ptr<Expression> pow(const std::shared_ptr<Expression> &base, const std::shared_ptr<Expression> &power) {
    if (!base || !power) return std::make_shared<Nan>();

    if (base->isNan()) return base;
    if (power->isNan()) return power;
    if (base->isZero() && power->isZero()) return std::make_shared<One>();
    if (base->isZero() || base->isOne()) return base;
    if (power->isOne()) return base;
    if (power->isZero()) return std::make_shared<One>();
    return std::make_shared<Power>(base, power);
}

std::shared_ptr<Expression> pow(const std::shared_ptr<Expression> &expr, const double &num) {
    return pow(expr, std::make_shared<Constant>(num));
}

std::shared_ptr<Expression> pow(const double &num, const std::shared_ptr<Expression> &expr) {
    return pow(std::make_shared<Constant>(num), expr);
}

std::shared_ptr<Expression> sqrt(const std::shared_ptr<Expression> &expr) {
    return pow(expr, std::make_shared<Constant>(1.0 / 2.0));
}
