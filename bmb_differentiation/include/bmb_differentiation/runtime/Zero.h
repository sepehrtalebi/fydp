#pragma once

#include <bmb_differentiation/runtime/Constant.h>
#include <memory>
#include <string>
#include <unordered_map>

class Zero : public Constant {
 public:
  static const ConstPtr INSTANCE;

  Zero(const Zero&) = delete;

  void operator=(const Zero&) = delete;

  [[nodiscard]] ExprPtr subs(
      const std::unordered_map<std::string, ExprPtr>& subs) const override;

  [[nodiscard]] ExprPtr simplify() const override;

  [[nodiscard]] std::string toStr() const override;

 private:
  Zero() noexcept : Constant(0) {}
};
