#include "Zero.h"

double Zero::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return 0;
}

std::shared_ptr<Expression> Zero::diff(const std::string & /** identifier **/) const {
    return std::make_shared<Zero>();
}

std::string Zero::toStr() const {
    return "0";
}

bool Zero::isZero() const {
    return true;
}
