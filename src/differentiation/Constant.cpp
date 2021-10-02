#include "Constant.h"
#include "Zero.h"

double Constant::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return value;
}

std::shared_ptr<Expression> Constant::diff(const std::string & /** identifier **/) const {
    return std::make_shared<Zero>();
}

std::shared_ptr<Expression> Constant::subs(const std::map<std::string, std::shared_ptr<Expression>> & /** subs **/) const {
    return std::make_shared<Constant>(value);
}

std::string Constant::toStr() const {
    return std::to_string(value);
}
