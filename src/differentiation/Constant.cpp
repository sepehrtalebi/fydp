#include "Constant.h"
#include "Zero.h"

double Constant::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return value;
}

ExprPtr Constant::diff(const std::string & /** identifier **/) const {
    return std::make_shared<Zero>();
}

ExprPtr Constant::subs(const std::map<std::string, ExprPtr> & /** subs **/) const {
    return std::make_shared<Constant>(value);
}

ExprPtr Constant::simplify() const {
    return std::make_shared<Constant>(value);
}

std::string Constant::toStr() const {
    return std::to_string(value);
}

double Constant::getValue() const {
    return value;
}
