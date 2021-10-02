#include "UnaryOperator.h"
#include "Constant.h"

double UnaryOperator::evaluate(const std::map<std::string, double> &variables) const {
    return call(operand->evaluate(variables));
}

ExprPtr UnaryOperator::diff(const std::string &id) const {
    return derivative(operand) * operand->diff(id);
}

ExprPtr UnaryOperator::subs(const std::map<std::string, ExprPtr> & subs) const {
    return call(operand->subs(subs));
}

ExprPtr UnaryOperator::simplify() const {
    // try evaluating constants
    std::shared_ptr<Constant> operand_const = std::dynamic_pointer_cast<Constant>(operand);
    if (operand_const) {
        return Constant::make(call(operand_const->getValue()));
    }
    return call(operand->simplify());
}

std::string UnaryOperator::toStr() const {
    return toStrWrapper(operand->toStr());
}

int UnaryOperator::nodeCount() const {
    return 1 + operand->nodeCount();
}
