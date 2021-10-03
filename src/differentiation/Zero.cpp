#include "Zero.h"

const ConstPtr Zero::INSTANCE = std::shared_ptr<Zero>(new Zero()); // NOLINT(cert-err58-cpp)

ExprPtr Zero::subs(const std::map<std::string, ExprPtr> & /** subs **/) const {
    return Zero::INSTANCE;
}

ExprPtr Zero::simplify() const {
    return Zero::INSTANCE;
}

std::string Zero::toStr() const {
    return "0";
}
