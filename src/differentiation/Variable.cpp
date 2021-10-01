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

std::string Variable::toStr() const {
    return identifier;
}
