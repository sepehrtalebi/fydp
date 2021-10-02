#include "Nan.h"
#include "Zero.h"
#include <cmath>

double Nan::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return std::nan("1");
}

std::shared_ptr<Expression> Nan::diff(const std::string & /** identifier **/) const {
    return std::make_shared<Nan>();
}

std::shared_ptr<Expression> Nan::subs(const std::map<std::string, std::shared_ptr<Expression>> & /** subs **/) const {
    return std::make_shared<Nan>();
}

std::shared_ptr<Expression> Nan::simplify() const {
    return std::make_shared<Nan>();
}

std::string Nan::toStr() const {
    return "Nan";
}

bool Nan::isNan() const {
    return true;
}
