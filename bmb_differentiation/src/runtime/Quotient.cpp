#include "bmb_differentiation/runtime/Quotient.h"
#include <bmb_differentiation/runtime/Constant.h>
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_differentiation/runtime/Zero.h>
#include <memory>
#include <string>

ExprPtr Quotient::diff(const std::string& id) const {
  return (first->diff(id) * second - second->diff(id) * first) /
         (second * second);
}

ExprPtr Quotient::simplify() const {
  std::shared_ptr<Variable> first_var =
      std::dynamic_pointer_cast<Variable>(first);
  std::shared_ptr<Variable> second_var =
      std::dynamic_pointer_cast<Variable>(second);
  if (first_var && second_var &&
      first_var->getIdentifier() == second_var->getIdentifier()) {
    return One::INSTANCE;
  }

  return BinaryOperator::simplify();
}

std::string Quotient::toStr() const {
  return "(" + first->toStr() + " / " + second->toStr() + ")";
}

double Quotient::call(const double& first, const double& second) const {
  return first / second;
}

ExprPtr Quotient::call(const ExprPtr& first, const ExprPtr& second) const {
  return first / second;
}

std::string Quotient::type() const { return "/"; }

ExprPtr operator/(const ExprPtr& num, const ExprPtr& den) {
  if (!num && !den) return One::INSTANCE;
  if (!num) return den;
  if (!den) return num;

  if (num == Nan::INSTANCE) return num;
  if (den == Nan::INSTANCE) return den;
  if (den == Zero::INSTANCE) return Nan::INSTANCE;
  if (den == One::INSTANCE) return num;
  if (num == Zero::INSTANCE) return num;
  // no special case needed if num->isOne()
  return std::make_shared<Quotient>(num, den);
}

ExprPtr operator/(const ExprPtr& expr, const double& num) {
  return expr / Constant::make(num);
}

ExprPtr operator/(const double& num, const ExprPtr& expr) {
  return Constant::make(num) / expr;
}

void operator/=(ExprPtr& expr1, const ExprPtr& expr2) { expr1 = expr1 / expr2; }

void operator/=(ExprPtr& expr, const double& num) { expr = expr / num; }

void operator/=(const double& num, ExprPtr& expr) { expr = num / expr; }
