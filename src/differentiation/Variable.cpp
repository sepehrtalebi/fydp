#include "Variable.h"
#include "Zero.h"
#include "One.h"

double Variable::evaluate(const std::map<std::string, double> &variables) const {
    return variables.at(identifier);
}

std::shared_ptr<Expression> Variable::diff(const std::string &id) const {
    if (identifier == id) return std::make_shared<One>();
    return std::make_shared<Zero>();
}

std::shared_ptr<Expression> Variable::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    auto it = subs.find(identifier);
    if (it == subs.end()) return std::make_shared<Variable>(identifier);
    return it->second;
}

std::string Variable::toStr() const {
    return identifier;
}
