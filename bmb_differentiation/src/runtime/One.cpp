#include "bmb_differentiation/runtime/One.h"

#include <string>
#include <unordered_map>

const ConstPtr One::INSTANCE = std::shared_ptr<One>(new One()); // NOLINT(cert-err58-cpp)

ExprPtr One::subs(const std::unordered_map<std::string, ExprPtr> & /** subs **/) const {
    return One::INSTANCE;
}

ExprPtr One::simplify() const {
    return One::INSTANCE;
}

std::string One::toStr() const {
    return "1";
}
