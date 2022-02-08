#include "bmb_differentiation/runtime/Zero.h"

#include <unordered_map>
#include <string>

const ConstPtr Zero::INSTANCE = std::shared_ptr<Zero>(new Zero()); // NOLINT(cert-err58-cpp)

ExprPtr Zero::subs(const std::unordered_map<std::string, ExprPtr> & /** subs **/) const {
    return Zero::INSTANCE;
}

ExprPtr Zero::simplify() const {
    return Zero::INSTANCE;
}

std::string Zero::toStr() const {
    return "0";
}
