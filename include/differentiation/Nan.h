#pragma once

#include "Constant.h"

#include <map>
#include <memory>
#include <string>
#include <limits>

class Nan : public Constant {
public:
    static const ConstPtr INSTANCE;

    Nan(const Nan &) = delete;

    void operator=(const Nan &) = delete;

    ExprPtr diff(const std::string &identifier) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

private:
    Nan() noexcept : Constant(std::numeric_limits<double>::quiet_NaN()) {}
};
