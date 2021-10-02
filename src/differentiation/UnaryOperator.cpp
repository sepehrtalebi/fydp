#include "UnaryOperator.h"
#include "Constant.h"

double UnaryOperator::evaluate(const std::map<std::string, double> &variables) const {
    return call(operand->evaluate(variables));
}

std::shared_ptr<Expression> UnaryOperator::diff(const std::string &id) const {
    return derivative(operand) * operand->diff(id);
}

std::shared_ptr<Expression> UnaryOperator::subs(const std::map<std::string, std::shared_ptr<Expression>> & subs) const {
    return call(operand->subs(subs));
}

std::shared_ptr<Expression> UnaryOperator::simplify() const {
    // try evaluating constants
    std::shared_ptr<Constant> operand_const = std::dynamic_pointer_cast<Constant>(operand);
    if (operand_const) {
        return std::make_shared<Constant>(call(operand_const->getValue()));
    }
    return call(operand->simplify());
}

std::string UnaryOperator::toStr() const {
    return toStrWrapper(operand->toStr());
}
