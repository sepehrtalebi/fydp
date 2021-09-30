#include "Constant.h"

double Constant::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return value;
}

Expression *Constant::diff(const std::string & /** identifier **/) const {
    return new Constant(0);
}

std::string Constant::toStr() const {
    return std::to_string(value);
}

Expression *Constant::copy() const {
    return new Constant(value);
}
