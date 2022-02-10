#include "bmb_differentiation/runtime/Exp.h"
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Zero.h>
#include <cmath>
#include <memory>
#include <string>

double Exp::call(const double& operand) const { return exp(operand); }

ExprPtr Exp::call(const ExprPtr& operand) const { return exp(operand); }

ExprPtr Exp::derivative(const ExprPtr& expr) const { return exp(expr); }

std::string Exp::toStrWrapper(const std::string& operandString) const {
  return "exp(" + operandString + ")";
}

ExprPtr exp(const ExprPtr& expr) {
  if (!expr) return Nan::INSTANCE;
  if (expr == Zero::INSTANCE) return One::INSTANCE;
  return std::make_shared<Exp>(expr);
}
