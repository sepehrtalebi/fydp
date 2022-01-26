#include "bmb_differentiation/runtime/Variable.h"

#include <bmb_differentiation/runtime/Zero.h>
#include <bmb_differentiation/runtime/One.h>

#include <map>
#include <memory>
#include <string>

std::shared_ptr<Variable> Variable::make(const std::string &id) {
    return std::make_shared<Variable>(id);
}

double Variable::evaluate(const std::map<std::string, double> &variables) const {
    return variables.at(identifier);
}

ExprPtr Variable::diff(const std::string &id) const {
    if (identifier == id) return One::INSTANCE;
    return Zero::INSTANCE;
}

ExprPtr Variable::subs(const std::map<std::string, ExprPtr> & subs) const {
    auto it = subs.find(identifier);
    if (it == subs.end()) return Variable::make(identifier);
    return it->second;
}

ExprPtr Variable::simplify() const {
    return Variable::make(identifier);
}

std::string Variable::toStr() const {
    return identifier;
}

std::string Variable::getIdentifier() const {
    return identifier;
}
