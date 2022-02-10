#include "bmb_differentiation/runtime/Constant.h"
#include <bmb_differentiation/runtime/Nan.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Zero.h>
#include <cmath>
#include <string>
#include <unordered_map>

ConstPtr Constant::make(const double& value) {
  if (value == 0.) return Zero::INSTANCE;
  if (value == 1.) return One::INSTANCE;
  if (std::isnan(value)) return Nan::INSTANCE;
  return std::shared_ptr<Constant>(new Constant(value));
}

double Constant::evaluate(
    const std::unordered_map<std::string, double>& /** variables **/) const {
  return value;
}

ExprPtr Constant::diff(const std::string& /** identifier **/) const {
  return Zero::INSTANCE;
}

ExprPtr Constant::subs(
    const std::unordered_map<std::string, ExprPtr>& /** subs **/) const {
  return make(value);
}

ExprPtr Constant::simplify() const { return make(value); }

std::string Constant::toStr() const { return std::to_string(value); }

double Constant::getValue() const { return value; }
