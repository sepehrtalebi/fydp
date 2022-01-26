#pragma once

#include <bmb_differentiation/runtime/Expression.h>

#include <map>
#include <memory>
#include <string>
#include <utility>

class Variable : public Expression {
private:
    std::string identifier;
public:
    explicit Variable(std::string id) : identifier(std::move(id)) {}

    static std::shared_ptr<Variable> make(const std::string &id);

    [[nodiscard]] double evaluate(const std::map<std::string, double> &variables) const override;

    [[nodiscard]] ExprPtr diff(const std::string &id) const override;

    [[nodiscard]] ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

    [[nodiscard]] std::string getIdentifier() const;
};
