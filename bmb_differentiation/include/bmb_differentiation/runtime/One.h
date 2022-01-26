#pragma once

#include <bmb_differentiation/runtime/Constant.h>

#include <map>
#include <memory>
#include <string>

class One : public Constant {
public:
    static const ConstPtr INSTANCE;

    One(const One &) = delete;

    void operator=(const One &) = delete;

    [[nodiscard]] ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

private:
    One() noexcept : Constant(1) {}
};
