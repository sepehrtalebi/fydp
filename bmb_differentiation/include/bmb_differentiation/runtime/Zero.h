#pragma once

#include <bmb_differentiation/runtime/Constant.h>

#include <map>
#include <memory>
#include <string>

class Zero : public Constant {
public:
    static const ConstPtr INSTANCE;

    Zero(const Zero &) = delete;

    void operator=(const Zero &) = delete;

    [[nodiscard]] ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

private:
    Zero() noexcept : Constant(0) {}
};
