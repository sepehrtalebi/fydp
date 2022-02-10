#include "bmb_differentiation/runtime/Product.h"
#include <bmb_differentiation/runtime/Constant.h>
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_differentiation/runtime/Zero.h>
#include <memory>
#include <string>

ExprPtr Product::diff(const std::string& id) const {
  return first->diff(id) * second + second->diff(id) * first;
}

ExprPtr Product::simplify() const {
  std::shared_ptr<Variable> first_var =
      std::dynamic_pointer_cast<Variable>(first);
  std::shared_ptr<Variable> second_var =
      std::dynamic_pointer_cast<Variable>(second);
  if (first_var && second_var &&
      first_var->getIdentifier() == second_var->getIdentifier()) {
    return pow(first_var, 2);
  }

  return BinaryOperator::simplify();
}

std::string Product::toStr() const {
  return "(" + first->toStr() + " * " + second->toStr() + ")";
}

double Product::call(const double& first, const double& second) const {
  return first * second;
}

ExprPtr Product::call(const ExprPtr& first, const ExprPtr& second) const {
  return first * second;
}

std::string Product::type() const { return "*"; }

bool Product::isAssociative() const { return true; }

bool Product::isCommutative() const { return true; }

bool Product::isDistributiveOn(const std::string& type) const {
  return type == "+" || type == "-";
}

std::shared_ptr<Expression> operator*(
    const std::shared_ptr<Expression>& expr1,
    const std::shared_ptr<Expression>& expr2) {
  if (!expr1 && !expr2) return One::INSTANCE;
  if (!expr1) return expr2;
  if (!expr2) return expr1;

  if (expr1 == Nan::INSTANCE) return expr1;
  if (expr2 == Nan::INSTANCE) return expr2;
  if (expr1 == Zero::INSTANCE) return expr1;
  if (expr2 == Zero::INSTANCE) return expr2;
  if (expr1 == One::INSTANCE) return expr2;
  if (expr2 == One::INSTANCE) return expr1;
  return std::make_shared<Product>(expr1, expr2);
}

std::shared_ptr<Expression> operator*(const std::shared_ptr<Expression>& expr,
                                      const double& num) {
  return Constant::make(num) * expr;
}

std::shared_ptr<Expression> operator*(const double& num,
                                      const std::shared_ptr<Expression>& expr) {
  return Constant::make(num) * expr;
}

void operator*=(std::shared_ptr<Expression>& expr1,
                const std::shared_ptr<Expression>& expr2) {
  expr1 = expr1 * expr2;
}

void operator*=(std::shared_ptr<Expression>& expr, const double& num) {
  expr = num * expr;
}

void operator*=(const double& num, std::shared_ptr<Expression>& expr) {
  expr = num * expr;
}
