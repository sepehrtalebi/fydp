#include "Variable.h"
#include "Zero.h"
#include "One.h"

double Variable::evaluate(const std::map<std::string, double> &variables) const {
    return variables.at(identifier);
}

ExprPtr Variable::diff(const std::string &id) const {
    if (identifier == id) return std::make_shared<One>();
    return std::make_shared<Zero>();
}

ExprPtr Variable::subs(const std::map<std::string, ExprPtr> & subs) const {
    auto it = subs.find(identifier);
    if (it == subs.end()) return std::make_shared<Variable>(identifier);
    return it->second;
}

ExprPtr Variable::simplify() const {
    return std::make_shared<Variable>(identifier);
}

std::string Variable::toStr() const {
    return identifier;
}

std::string Variable::getIdentifier() const {
    return identifier;
}
