#include "Log.h"
#include "Zero.h"
#include "Nan.h"
#include "Sum.h"

#include <cmath>

double Log::evaluate(const std::map<std::string, double> &variables) const {
    return log(value->evaluate(variables));
}

std::shared_ptr<Expression> Log::diff(const std::string &id) const {
    return value->diff(id) / value;
}

std::shared_ptr<Expression> Log::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return std::make_shared<Log>(value->subs(subs));
}

std::string Log::toStr() const {
    return "ln(" + value->toStr() + ")";
}

std::shared_ptr<Expression> log(const std::shared_ptr<Expression> &expr) {
    if (!expr || expr->isZero()) return std::make_shared<Nan>();
    if (expr->isOne()) return std::make_shared<Zero>();
    return std::make_shared<Log>(expr);
}
