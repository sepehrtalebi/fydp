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

std::shared_ptr<Expression> operator+(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2);

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2);

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression>& expr);

std::shared_ptr<Expression> operator*(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2);

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2);

std::shared_ptr<Expression> operator+(const double& num, const std::shared_ptr<Expression>& expr);

std::shared_ptr<Expression> operator-(const double& num, const std::shared_ptr<Expression>& expr);

std::shared_ptr<Expression> operator*(const double& num, const std::shared_ptr<Expression>& expr);

std::shared_ptr<Expression> operator/(const double& num, const std::shared_ptr<Expression>& expr);

std::shared_ptr<Expression> operator+(const std::shared_ptr<Expression>& expr, const double& num);

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression>& expr, const double& num);

std::shared_ptr<Expression> operator*(const std::shared_ptr<Expression>& expr, const double& num);

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression>& expr, const double& num);

void operator+=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

void operator-=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

void operator*=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

void operator/=(std::shared_ptr<Expression> &expr1, const std::shared_ptr<Expression> &expr2);

void operator+=(std::shared_ptr<Expression> &expr, const double &num);

void operator-=(std::shared_ptr<Expression> &expr, const double &num);

void operator*=(std::shared_ptr<Expression> &expr, const double &num);

void operator/=(std::shared_ptr<Expression> &expr, const double &num);

void operator+=(const double &num, std::shared_ptr<Expression> &expr);

void operator-=(const double &num, std::shared_ptr<Expression> &expr);

void operator*=(const double &num, std::shared_ptr<Expression> &expr);

void operator/=(const double &num, std::shared_ptr<Expression> &expr);
