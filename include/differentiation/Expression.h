#pragma once

#include <map>
#include <memory>
#include <string>

class Expression;

using ExprPtr = std::shared_ptr<Expression>;

class Expression {
    // Note that we can't declare variables of type Expression since it is abstract
    // Thus we also can't have them as elements in Vector
    // The workaround is to use ExprPtr in its place, and define global arithmetic operators to work with it
public:
    [[nodiscard]] virtual double evaluate(const std::map<std::string, double> &variables) const = 0;

    [[nodiscard]] virtual ExprPtr diff(const std::string &id) const = 0;

    [[nodiscard]] virtual ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const = 0;

    [[nodiscard]] virtual ExprPtr simplify() const = 0;

    [[nodiscard]] virtual std::string toStr() const = 0;

    [[nodiscard]] virtual unsigned int nodeCount() const;
};

// Declare all the operators that are defined in subclasses
// This ensures all these operators can be accessed, even if only Expression.h is #included

// Sum.cpp

ExprPtr operator+(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr operator+(const ExprPtr &expr, const double &num);

ExprPtr operator+(const double &num, const ExprPtr &expr);

void operator+=(ExprPtr &expr1, const ExprPtr &expr2);

void operator+=(ExprPtr &expr, const double &num);

void operator+=(const double &num, ExprPtr &expr);

// Difference.cpp

ExprPtr operator-(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr operator-(const ExprPtr &expr);

ExprPtr operator-(const ExprPtr &expr, const double &num);

ExprPtr operator-(const double &num, const ExprPtr &expr);

void operator-=(ExprPtr &expr1, const ExprPtr &expr2);

void operator-=(ExprPtr &expr, const double &num);

void operator-=(const double &num, ExprPtr &expr);

// Product.cpp

ExprPtr operator*(const ExprPtr& expr1, const ExprPtr& expr2);

ExprPtr operator*(const ExprPtr& expr, const double& num);

ExprPtr operator*(const double& num, const ExprPtr& expr);

void operator*=(ExprPtr &expr1, const ExprPtr &expr2);

void operator*=(ExprPtr &expr, const double &num);

void operator*=(const double &num, ExprPtr &expr);

// Quotient.cpp

ExprPtr operator/(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr operator/(const ExprPtr &expr, const double &num);

ExprPtr operator/(const double &num, const ExprPtr &expr);

void operator/=(ExprPtr &expr1, const ExprPtr &expr2);

void operator/=(ExprPtr &expr, const double &num);

void operator/=(const double &num, ExprPtr &expr);

// Power.cpp

ExprPtr pow(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr pow(const ExprPtr &expr, const double &num);

ExprPtr pow(const double &num, const ExprPtr &expr);

ExprPtr sqrt(const ExprPtr &expr);

// Log.cpp

ExprPtr log(const ExprPtr &expr);

// Sin.cpp

ExprPtr sin(const ExprPtr &expr);

// Cos.cpp

ExprPtr cos(const ExprPtr &expr);
