#pragma once

#include "Constant.h"

#include <map>
#include <memory>
#include <string>

class Zero : public Constant {
public:
    static const ConstPtr INSTANCE;

    Zero(const Zero &) = delete;

    void operator=(const Zero &) = delete;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

private:
    Zero() noexcept : Constant(0) {}
};
