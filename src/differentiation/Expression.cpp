#include "Expression.h"
#include "Zero.h"
#include "One.h"
#include "Nan.h"
#include "Constant.h"
#include "Sum.h"
#include "Difference.h"
#include "Product.h"
#include "Quotient.h"

bool Expression::isZero() const {
    return false;
}

bool Expression::isOne() const {
    return false;
}

bool Expression::isNan() const {
    return false;
}

std::shared_ptr<Expression> operator+(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2) {
    if (!expr1 && !expr2) return std::make_shared<Zero>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr1->isZero()) return expr2;
    if (expr2->isZero()) return expr1;
    return std::make_shared<Sum>(expr1, expr2);
}

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2) {
    if (!expr1 && !expr2) return std::make_shared<Zero>();
    if (!expr1) return std::make_shared<Product>(std::make_shared<Constant>(-1), expr2);
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr2->isZero()) return expr1;
    if (expr1->isZero()) return std::make_shared<Product>(std::make_shared<Constant>(-1), expr2);
    return std::make_shared<Difference>(expr1, expr2);
}

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression>& expr) {
    if (!expr) return std::make_shared<Zero>();

    if (expr->isNan() || expr->isZero()) return expr;
    return std::make_shared<Product>(std::make_shared<Constant>(-1), expr);
}

std::shared_ptr<Expression> operator*(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2) {
    if (!expr1 && !expr2) return std::make_shared<One>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr1->isZero()) return expr1;
    if (expr2->isZero()) return expr2;
    if (expr1->isOne()) return expr2;
    if (expr2->isOne()) return expr1;
    return std::make_shared<Product>(expr1, expr2);
}

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression>& expr1, const std::shared_ptr<Expression>& expr2) {
    if (!expr1 && !expr2) return std::make_shared<One>();
    if (!expr1) return expr2;
    if (!expr2) return expr1;

    if (expr1->isNan()) return expr1;
    if (expr2->isNan()) return expr2;
    if (expr2->isZero()) return std::make_shared<Nan>();
    if (expr2->isOne()) return expr1;
    if (expr1->isZero()) return expr1;
    // no special case needed if expr1->isOne()
    return std::make_shared<Quotient>(expr1, expr2);
}


std::shared_ptr<Expression> operator+(const double& num, const std::shared_ptr<Expression>& expr) {
    return std::make_shared<Constant>(num) + expr;
}

std::shared_ptr<Expression> operator-(const double& num, const std::shared_ptr<Expression>& expr) {
    return std::make_shared<Constant>(num) - expr;
}

std::shared_ptr<Expression> operator*(const double& num, const std::shared_ptr<Expression>& expr) {
    return std::make_shared<Constant>(num) * expr;
}

std::shared_ptr<Expression> operator/(const double& num, const std::shared_ptr<Expression>& expr)  {
    return std::make_shared<Constant>(num) / expr;
}

std::shared_ptr<Expression> operator+(const std::shared_ptr<Expression>& expr, const double& num) {
    return expr + std::make_shared<Constant>(num);
}

std::shared_ptr<Expression> operator-(const std::shared_ptr<Expression>& expr, const double& num) {
    return expr - std::make_shared<Constant>(num);
}

std::shared_ptr<Expression> operator*(const std::shared_ptr<Expression>& expr, const double& num) {
    return expr * std::make_shared<Constant>(num);
}

std::shared_ptr<Expression> operator/(const std::shared_ptr<Expression>& expr, const double& num) {
    return expr / std::make_shared<Constant>(num);
}
