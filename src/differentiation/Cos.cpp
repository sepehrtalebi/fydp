#include "Cos.h"
#include "One.h"
#include "Nan.h"
#include "Sum.h"
#include "Constant.h"

#include <cmath>

double Cos::evaluate(const std::map<std::string, double> &variables) const {
    return cos(value->evaluate(variables));
}

std::shared_ptr<Expression> Cos::diff(const std::string &id) const {
    return -sin(value) * value->diff(id);
}

std::shared_ptr<Expression> Cos::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return cos(value->subs(subs));
}

std::shared_ptr<Expression> Cos::simplify() const {
    // try evaluating constants
    std::shared_ptr<Constant> value_const = std::dynamic_pointer_cast<Constant>(value);
    if (value_const) {
        return std::make_shared<Constant>(cos(value_const->getValue()));
    }
    return cos(value->simplify());
}

std::string Cos::toStr() const {
    return "cos(" + value->toStr() + ")";
}

std::shared_ptr<Expression> cos(const std::shared_ptr<Expression> &expr) {
    if (!expr) return std::make_shared<Nan>();
    if (expr->isZero()) return std::make_shared<One>();
    return std::make_shared<Cos>(expr);
}
