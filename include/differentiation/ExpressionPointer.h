#pragma once

#include "Expression.h"

class ExpressionPointer {
private:
    Expression *pointer;
public:
    ExpressionPointer();

    explicit ExpressionPointer(Expression *pointer) : pointer(pointer) {}

    ~ExpressionPointer();

    double evaluate(const std::map<std::string, double> &variables) const;

    ExpressionPointer diff(const std::string &id) const;

    std::string toStr() const;

    ExpressionPointer operator+(const ExpressionPointer &other) const;
};
