#pragma once

#include <bmb_differentiation/runtime/Constant.h>

#include <map>
#include <memory>
#include <string>
#include <limits>

class Nan : public Constant {
public:
    static const ConstPtr INSTANCE;

    Nan(const Nan &) = delete;

    void operator=(const Nan &) = delete;

    [[nodiscard]] ExprPtr diff(const std::string &identifier) const override;

    [[nodiscard]] ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

private:
    Nan() noexcept : Constant(std::numeric_limits<double>::quiet_NaN()) {}
};
