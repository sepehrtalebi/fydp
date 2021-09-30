#include "ExpressionPointer.h"
#include "Sum.h"
#include "Constant.h"
#include "Variable.h"
#include <iostream>

ExpressionPointer::ExpressionPointer() {
    pointer = nullptr;
}

ExpressionPointer::~ExpressionPointer() {
    delete pointer;
}

double ExpressionPointer::evaluate(const std::map<std::string, double> &variables) const {
    return pointer->evaluate(variables);
}

ExpressionPointer ExpressionPointer::diff(const std::string &id) const {
    return ExpressionPointer{pointer->diff(id)};
}

std::string ExpressionPointer::toStr() const {
    return pointer->toStr();
}

ExpressionPointer ExpressionPointer::operator+(const ExpressionPointer &other) const {
    if (!pointer && !other.pointer) {
        std::cout << "Undefined behaviour! Returning zero." << std::endl;
        return ExpressionPointer{new Constant{0}};
    }
    if (!pointer) return ExpressionPointer{other.pointer->copy()};
    if (!other.pointer) return ExpressionPointer{pointer->copy()};
    std::cout << "almost stuff" << std::endl;
    Expression *pExpression = ((Variable*) pointer)->copy();
    std::cout << "stuff" << std::endl;
    return ExpressionPointer{new Sum{pExpression, other.pointer->copy()}};
}
