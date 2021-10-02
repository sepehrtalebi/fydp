#include "One.h"
#include "Zero.h"

double One::evaluate(const std::map<std::string, double> &/** variables **/) const {
    return 1;
}

ExprPtr One::diff(const std::string & /** identifier **/) const {
    return std::make_shared<Zero>();
}

ExprPtr One::subs(const std::map<std::string, ExprPtr> & /** subs **/) const {
    return std::make_shared<One>();
}

ExprPtr One::simplify() const {
    return std::make_shared<One>();
}

std::string One::toStr() const {
    return "1";
}

bool One::isOne() const {
    return true;
}
