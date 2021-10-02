#pragma once

#include "Constant.h"

#include <map>
#include <memory>
#include <string>

class One : public Constant {
public:
    static const ConstPtr INSTANCE;

    One(const One &) = delete;

    void operator=(const One &) = delete;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

private:
    One() noexcept : Constant(1) {}
};
