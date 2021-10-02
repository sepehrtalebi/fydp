#include "Zero.h"

double Zero::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return 0;
}

ExprPtr Zero::diff(const std::string & /** identifier **/) const {
    return std::make_shared<Zero>();
}

ExprPtr Zero::subs(const std::map<std::string, ExprPtr> & /** subs **/) const {
    return std::make_shared<Zero>();
}

ExprPtr Zero::simplify() const {
    return std::make_shared<Zero>();
}

std::string Zero::toStr() const {
    return "0";
}

bool Zero::isZero() const {
    return true;
}
