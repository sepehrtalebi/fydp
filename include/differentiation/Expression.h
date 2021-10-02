#pragma once

#include <map>
#include <memory>
#include <string>

class Expression {
    // Note that we can't declare variables of type Expression since it is abstract
    // Thus we also can't have them as elements in Vector
    // The workaround is to use std::shared_ptr<Expression> in its place, and defined global arithmetic operators to work with it
public:
    virtual double evaluate(const std::map<std::string, double> &variables) const = 0;

    virtual std::shared_ptr<Expression> diff(const std::string &id) const = 0;

    virtual std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const = 0;

    virtual std::string toStr() const = 0;

    virtual bool isZero() const;

    virtual bool isOne() const;

    virtual bool isNan() const;
};

// We need to #include all these sub-classes because they overload operators for std::shared_ptr<Expression>
#include "Sum.h"
#include "Difference.h"
#include "Product.h"
#include "Quotient.h"
#include "Power.h"
#include "Log.h"
