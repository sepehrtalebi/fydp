#include "bmb_differentiation/runtime/Log.h"
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Zero.h>
#include <cmath>
#include <memory>
#include <string>

double Log::call(const double& operand) const { return log(operand); }

ExprPtr Log::call(const ExprPtr& operand) const { return log(operand); }

ExprPtr Log::derivative(const ExprPtr& expr) const { return 1 / expr; }

std::string Log::toStrWrapper(const std::string& operandString) const {
  return "ln(" + operandString + ")";
}

ExprPtr log(const ExprPtr& expr) {
  if (!expr || expr == Zero::INSTANCE) return Nan::INSTANCE;
  if (expr == One::INSTANCE) return Zero::INSTANCE;
  return std::make_shared<Log>(expr);
}
