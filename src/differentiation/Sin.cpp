#include "Sin.h"
#include "Zero.h"
#include "Nan.h"
#include "Sum.h"
#include "Constant.h"

#include <cmath>

double Sin::evaluate(const std::map<std::string, double> &variables) const {
    return sin(value->evaluate(variables));
}

std::shared_ptr<Expression> Sin::diff(const std::string &id) const {
    return cos(value) * value->diff(id);
}

std::shared_ptr<Expression> Sin::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return sin(value->subs(subs));
}

std::shared_ptr<Expression> Sin::simplify() const {
    // try evaluating constants
    std::shared_ptr<Constant> value_const = std::dynamic_pointer_cast<Constant>(value);
    if (value_const) {
        return std::make_shared<Constant>(sin(value_const->getValue()));
    }
    return sin(value->simplify());
}

std::string Sin::toStr() const {
    return "sin(" + value->toStr() + ")";
}

std::shared_ptr<Expression> sin(const std::shared_ptr<Expression> &expr) {
    if (!expr) return std::make_shared<Nan>();
    if (expr->isZero()) return std::make_shared<Zero>();
    return std::make_shared<Sin>(expr);
}
