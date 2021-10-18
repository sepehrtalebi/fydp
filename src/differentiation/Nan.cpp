#include "Nan.h"

const ConstPtr Nan::INSTANCE = std::shared_ptr<Nan>(new Nan()); // NOLINT(cert-err58-cpp)

ExprPtr Nan::diff(const std::string & /** identifier **/) const {
    return Nan::INSTANCE;
}

ExprPtr Nan::subs(const std::map<std::string, ExprPtr> & /** subs **/) const {
    return Nan::INSTANCE;
}

ExprPtr Nan::simplify() const {
    return Nan::INSTANCE;
}

std::string Nan::toStr() const {
    return "Nan";
}
